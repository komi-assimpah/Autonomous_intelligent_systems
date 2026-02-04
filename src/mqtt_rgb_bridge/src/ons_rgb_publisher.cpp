cat rgb_publisher.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

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
    this->declare_parameter<int>("fps", 10);              // hotspot-safe
    this->declare_parameter<int>("jpeg_quality", 50);     // 40-60 good
    this->declare_parameter<std::string>("topic", "/camera/rgb/image_compressed");

    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    fps_ = this->get_parameter("fps").as_int();
    jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();
    topic_ = this->get_parameter("topic").as_string();

    // Publisher: BEST_EFFORT (SensorDataQoS)
    image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      topic_, rclcpp::SensorDataQoS()
    );

    // RealSense config: COLOR only
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_RGB8, fps_);

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
      "✅ RealSense RGB COMPRESSED publisher started: %s (%dx%d @ %dfps, jpeg_q=%d, best_effort)",
      topic_.c_str(), width_, height_, fps_, jpeg_quality_);
  }

private:
  void tick()
  {
    rs2::frameset fs;
    if (!pipeline_.poll_for_frames(&fs)) {
      return;
    }

    rs2::video_frame color = fs.get_color_frame();
    if (!color) return;

    publish_jpeg(color);
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

  // RealSense
  rs2::pipeline pipeline_;
  rs2::pipeline_profile profile_;

  // ROS2
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr image_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Params
  int width_{424};
  int height_{240};
  int fps_{10};
  int jpeg_quality_{50};
  std::string topic_{"/camera/rgb/image_compressed"};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealSenseRgbPublisher>());
  rclcpp::shutdown();
  return 0;
}