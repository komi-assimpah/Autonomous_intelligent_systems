#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

class RealSenseRgbPublisher : public rclcpp::Node
{
public:
  RealSenseRgbPublisher()
  : Node("realsense_rgb_publisher")
  {
    // Params (easy tuning)
    this->declare_parameter<int>("width", 424);
    this->declare_parameter<int>("height", 240);
    this->declare_parameter<int>("fps", 10);
    this->declare_parameter<int>("jpeg_quality", 50);
    this->declare_parameter<bool>("enable_depth", false);
    this->declare_parameter<std::string>("topic", "/camera/rgb/image_compressed");
    this->declare_parameter<std::string>("depth_topic", "/camera/depth/image_raw");

    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    fps_ = this->get_parameter("fps").as_int();
    jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();
    enable_depth_ = this->get_parameter("enable_depth").as_bool();
    topic_ = this->get_parameter("topic").as_string();
    depth_topic_ = this->get_parameter("depth_topic").as_string();

    // Publisher: BEST_EFFORT (SensorDataQoS)
    auto qos = rclcpp::SensorDataQoS();
    image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(topic_, qos);
    
    if (enable_depth_) {
      depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(depth_topic_, qos);
      camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(depth_topic_ + "/camera_info", qos);
    }

    // RealSense config: COLOR + DEPTH (if depth enabled)
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_RGB8, fps_);
    
    if (enable_depth_) {
      cfg.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);
    }

    profile_ = pipeline_.start(cfg);

    // Warm-up
    for (int i = 0; i < 10; ++i) {
      pipeline_.wait_for_frames();
    }

    // Timer based on fps
    int period_ms = static_cast<int>(1000.0 / std::max(1, fps_));
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&RealSenseRgbPublisher::tick, this)
    );

    RCLCPP_INFO(this->get_logger(),
      "✅ RealSense publisher started: %s (%dx%d @ %dfps, jpeg_q=%d, depth=%s)",
      topic_.c_str(), width_, height_, fps_, jpeg_quality_,
      enable_depth_ ? "ON" : "OFF");
  }

private:
  void tick()
  {
    rs2::frameset fs;
    if (!pipeline_.poll_for_frames(&fs)) {
      return;
    }

    rs2::video_frame color = fs.get_color_frame();
    if (color) {
      publish_jpeg(color);
    }
    
    if (enable_depth_) {
      rs2::depth_frame depth = fs.get_depth_frame();
      if (depth) {
        publish_depth(depth);
      }
    }
  }

  void publish_jpeg(const rs2::video_frame &color)
  {
    const int w = color.get_width();
    const int h = color.get_height();

    // RealSense gives BGR8 because we configured RS2_FORMAT_BGR8
    cv::Mat rgb(cv::Size(w, h), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat bgr;
    cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
    
    // Encode JPEG
    std::vector<uchar> buf;
    std::vector<int> params = { cv::IMWRITE_JPEG_QUALITY, jpeg_quality_ };
    if (!cv::imencode(".jpg", bgr, buf, params)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "JPEG encode failed");
      return;
    }

    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "camera_color_frame";
    msg.format = "jpeg";
    msg.data = std::move(buf);

    image_pub_->publish(msg);
  }

  void publish_depth(const rs2::depth_frame &depth)
  {
    const int w = depth.get_width();
    const int h = depth.get_height();
    
    // Create depth image message (16-bit)
    auto depth_msg = sensor_msgs::msg::Image();
    depth_msg.header.stamp = this->now();
    depth_msg.header.frame_id = "camera_depth_frame";
    depth_msg.height = h;
    depth_msg.width = w;
    depth_msg.encoding = "16UC1";  // 16-bit unsigned, 1 channel
    depth_msg.is_bigendian = false;
    depth_msg.step = w * sizeof(uint16_t);
    
    // Copy depth data
    const uint16_t* depth_data = reinterpret_cast<const uint16_t*>(depth.get_data());
    depth_msg.data.resize(h * w * sizeof(uint16_t));
    std::memcpy(depth_msg.data.data(), depth_data, depth_msg.data.size());
    
    depth_pub_->publish(depth_msg);
    
    // Publish camera info for 3D reconstruction
    auto depth_intrin = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
    
    auto camera_info_msg = sensor_msgs::msg::CameraInfo();
    camera_info_msg.header = depth_msg.header;
    camera_info_msg.width = w;
    camera_info_msg.height = h;
    camera_info_msg.distortion_model = "plumb_bob";
    
    // Intrinsic matrix
    camera_info_msg.k[0] = depth_intrin.fx;
    camera_info_msg.k[2] = depth_intrin.ppx;
    camera_info_msg.k[4] = depth_intrin.fy;
    camera_info_msg.k[5] = depth_intrin.ppy;
    camera_info_msg.k[8] = 1.0;
    
    // Projection matrix
    camera_info_msg.p[0] = depth_intrin.fx;
    camera_info_msg.p[2] = depth_intrin.ppx;
    camera_info_msg.p[5] = depth_intrin.fy;
    camera_info_msg.p[6] = depth_intrin.ppy;
    camera_info_msg.p[10] = 1.0;
    
    camera_info_pub_->publish(camera_info_msg);
  }

  // RealSense
  rs2::pipeline pipeline_;
  rs2::pipeline_profile profile_;

  // ROS2
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Params
  int width_{424};
  int height_{240};
  int fps_{10};
  int jpeg_quality_{50};
  bool enable_depth_{false};
  std::string topic_{"/camera/rgb/image_compressed"};
  std::string depth_topic_{"/camera/depth/image_raw"};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealSenseRgbPublisher>());
  rclcpp::shutdown();
  return 0;
}
