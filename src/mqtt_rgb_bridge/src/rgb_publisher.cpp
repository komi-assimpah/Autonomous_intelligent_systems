#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

/*
Un type de donnee qui contient 2 imges:
1. Image RGB "raw" sans traitement open cv
2. Image depth map

Compression du tout et eemettre sur topic.
*/

class RealSenseRgbdPublisher : public rclcpp::Node
{
public:
  RealSenseRgbdPublisher()
  : Node("realsense_rgbd_publisher")
  {
    // Params (easy tuning)
    this->declare_parameter<int>("width", 424);
    this->declare_parameter<int>("height", 240);
    this->declare_parameter<int>("fps", 6);
    this->declare_parameter<int>("jpeg_quality", 50);     // 0-100 JPEG quality
    this->declare_parameter<int>("depth_quality", 1);    // 0-9 PNG compression level (1=fastest)
    this->declare_parameter<std::string>("topic", "/camera/rgbd/compressed");

    width_ = this->get_parameter("width").as_int();
    height_ = this->get_parameter("height").as_int();
    fps_ = this->get_parameter("fps").as_int();
    jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();
    depth_quality_ = this->get_parameter("depth_quality").as_int();
    topic_ = this->get_parameter("topic").as_string();

    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      topic_, rclcpp::SensorDataQoS()
    );

    // RealSense config: Enable BOTH streams
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, width_, height_, RS2_FORMAT_RGB8, fps_);
    cfg.enable_stream(RS2_STREAM_DEPTH, width_, height_, RS2_FORMAT_Z16, fps_);

    RCLCPP_INFO(this->get_logger(), "Connecting to RealSense...");
    try {
        profile_ = pipeline_.start(cfg);

    } catch (const rs2::error & e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline: %s", e.what());
        return;
    }

    // Warm-up
    for (int i = 0; i < 5; ++i) {
      rs2::frameset warmup_fs;
      pipeline_.poll_for_frames(&warmup_fs);
    }

    // Timer
    int period_ms = static_cast<int>(1000.0 / fps_);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&RealSenseRgbdPublisher::tick, this)
    );

    RCLCPP_INFO(this->get_logger(),
      "✅ RealSense RGB-D Packed Publisher started: %s (%dx%d @ %dfps)",
      topic_.c_str(), width_, height_, fps_);
  }

private:
  void tick()
  {
    rs2::frameset fs;
    if (!pipeline_.poll_for_frames(&fs)) {
        return;
    }

    rs2::video_frame color_frame = fs.get_color_frame();
    rs2::depth_frame depth_frame = fs.get_depth_frame();

    if (!color_frame || !depth_frame) {
        return; 
    }

    publish_packed(color_frame, depth_frame);
  }

  void publish_packed(const rs2::video_frame& color, const rs2::depth_frame& depth)
  {
    // 1. Convert Color to OpenCV and Compress (JPEG)
    cv::Mat rgb_mat(cv::Size(width_, height_), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat bgr_mat;
    cv::cvtColor(rgb_mat, bgr_mat, cv::COLOR_RGB2BGR); // OpenCV uses BGR

    std::vector<uchar> rgb_buf;
    std::vector<int> jpeg_params = { cv::IMWRITE_JPEG_QUALITY, jpeg_quality_ };
    if (!cv::imencode(".jpg", bgr_mat, rgb_buf, jpeg_params)) {
        RCLCPP_ERROR(this->get_logger(), "RGB compress failed");
        return;
    }

    // 2. Convert Depth to OpenCV and Compress (PNG)
    // Depth is 16-bit (Z16)
    cv::Mat depth_mat(cv::Size(width_, height_), CV_16UC1, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
    
    std::vector<uchar> depth_buf;
    // PNG is lossless but slow. Use compression level 1 (fastest)
    std::vector<int> png_params = { cv::IMWRITE_PNG_COMPRESSION, depth_quality_ };
    if (!cv::imencode(".png", depth_mat, depth_buf, png_params)) {
         RCLCPP_ERROR(this->get_logger(), "Depth compress failed");
         return;
    }

    // 3. Pack into one binary blob
    // [RGB_SIZE (4 bytes)] [DEPTH_SIZE (4 bytes)] [RGB_DATA] [DEPTH_DATA]
    uint32_t rgb_size = static_cast<uint32_t>(rgb_buf.size());
    uint32_t depth_size = static_cast<uint32_t>(depth_buf.size());
    
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "camera_color_frame"; // frames are aligned
    msg.format = "rgbd;jpeg;png"; // Custom format identifier

    msg.data.resize(sizeof(uint32_t) * 2 + rgb_size + depth_size);
    uint8_t* ptr = msg.data.data();

    // Copy Header
    std::memcpy(ptr, &rgb_size, sizeof(uint32_t)); ptr += sizeof(uint32_t);
    std::memcpy(ptr, &depth_size, sizeof(uint32_t)); ptr += sizeof(uint32_t);

    // Copy Data
    std::memcpy(ptr, rgb_buf.data(), rgb_size); ptr += rgb_size;
    std::memcpy(ptr, depth_buf.data(), depth_size);

    pub_->publish(msg);
  }

  // RealSense
  rs2::pipeline pipeline_;
  rs2::pipeline_profile profile_;

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Config
  int width_, height_, fps_, jpeg_quality_, depth_quality_;
  std::string topic_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RealSenseRgbdPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
