#include <atomic>
#include <mutex>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

class CameraRtspPublisherNode : public rclcpp::Node
{
public:
  CameraRtspPublisherNode()
  : Node("camera_rtsp_publisher"), writer_initialized_(false)
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/kordon001/camera/image_raw");
    rtsp_url_ = declare_parameter<std::string>("rtsp_url", "rtsp://127.0.0.1:8554/kordon001");
    bitrate_kbps_ = declare_parameter<int>("bitrate_kbps", 2500);
    keyint_ = declare_parameter<int>("keyint", 30);
    expected_fps_ = declare_parameter<double>("fps", 30.0);

    if (bitrate_kbps_ <= 0) {
      throw std::invalid_argument("bitrate_kbps must be > 0");
    }
    if (keyint_ <= 0) {
      throw std::invalid_argument("keyint must be > 0");
    }
    if (expected_fps_ <= 0.0) {
      throw std::invalid_argument("fps must be > 0");
    }

    sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&CameraRtspPublisherNode::on_image, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "camera_rtsp_publisher started: input_topic=%s rtsp_url=%s bitrate=%d kbps keyint=%d fps=%.2f",
      input_topic_.c_str(), rtsp_url_.c_str(), bitrate_kbps_, keyint_, expected_fps_);
  }

private:
  void on_image(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat bgr;
    try {
      cv_bridge::CvImageConstPtr cv_ptr;
      if (msg->encoding == sensor_msgs::image_encodings::BGR8) {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
      } else {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      bgr = cv_ptr->image;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "cv_bridge conversion failed: %s", e.what());
      return;
    }

    if (bgr.empty()) {
      return;
    }

    {
      std::lock_guard<std::mutex> lock(writer_mutex_);
      if (!writer_initialized_) {
        if (!init_writer(bgr.cols, bgr.rows)) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to initialize GStreamer writer");
          return;
        }
      }

      if (!writer_.isOpened()) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000, "GStreamer writer is not opened");
        writer_initialized_ = false;
        return;
      }
      writer_.write(bgr);
    }
  }

  bool init_writer(int width, int height)
  {
    const std::string pipeline =
      "appsrc format=time is-live=true do-timestamp=true "
      "! videoconvert "
      "! x264enc tune=zerolatency speed-preset=veryfast bitrate=" + std::to_string(bitrate_kbps_) +
      " key-int-max=" + std::to_string(keyint_) +
      " byte-stream=true "
      "! h264parse config-interval=1 "
      "! rtspclientsink location=" + rtsp_url_ + " protocols=tcp";

    RCLCPP_INFO(get_logger(), "Opening GStreamer pipeline: %s", pipeline.c_str());

    writer_initialized_ = writer_.open(
      pipeline,
      cv::CAP_GSTREAMER,
      0,
      expected_fps_,
      cv::Size(width, height),
      true);

    if (!writer_initialized_) {
      RCLCPP_ERROR(get_logger(), "Could not open GStreamer pipeline for RTSP publishing");
    }
    return writer_initialized_;
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  std::string input_topic_;
  std::string rtsp_url_;
  int bitrate_kbps_;
  int keyint_;
  double expected_fps_;

  cv::VideoWriter writer_;
  bool writer_initialized_;
  std::mutex writer_mutex_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraRtspPublisherNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
