#include <algorithm>
#include <chrono>
#include <cctype>
#include <filesystem>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "block_interfaces/srv/detect_block.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "visualization_msgs/msg/image_marker.hpp"

namespace
{
using DetectBlock = block_interfaces::srv::DetectBlock;
using CompressedImage = sensor_msgs::msg::CompressedImage;
using ImageMarker = visualization_msgs::msg::ImageMarker;

enum class Color { Red, Blue, Green };

std::optional<Color> parse_color(std::string text)
{
  const auto first = text.find_first_not_of(" \t\r\n\v\f");
  if (first == std::string::npos) {
    return std::nullopt;
  }
  text = text.substr(first, text.find_last_not_of(" \t\r\n\v\f") - first + 1);
  std::transform(text.begin(), text.end(), text.begin(),
    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

  if (text == "red" || text == "红" || text == "红色") {
    return Color::Red;
  }
  if (text == "blue" || text == "蓝" || text == "蓝色") {
    return Color::Blue;
  }
  if (text == "green" || text == "绿" || text == "绿色") {
    return Color::Green;
  }
  return std::nullopt;
}

std::optional<cv::Rect> detect(const cv::Mat & hsv, Color color)
{
  cv::Mat mask;
  const auto hue_mask = [&hsv](int low, int high) {
      cv::Mat result;
      cv::inRange(hsv, cv::Scalar(low, 95, 45), cv::Scalar(high, 255, 255), result);
      return result;
    };

  // OpenCV 的 H 范围是 0–179，红色横跨两端。
  switch (color) {
    case Color::Red:
      cv::bitwise_or(hue_mask(0, 12), hue_mask(167, 179), mask);
      break;
    case Color::Blue:
      mask = hue_mask(88, 125);
      break;
    case Color::Green:
      mask = hue_mask(36, 85);
      break;
  }

  const auto kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, {7, 7});
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel, {-1, -1}, 2);
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  double best_area = 0.0;
  const auto min_area = static_cast<double>(hsv.total()) * 0.002;
  std::optional<cv::Rect> best;
  for (const auto & contour : contours) {
    const auto area = cv::contourArea(contour);
    if (area < min_area || area <= best_area) {
      continue;
    }
    const auto box = cv::boundingRect(contour);
    const auto aspect = static_cast<double>(box.width) / box.height;
    const auto fill = area / (static_cast<double>(box.width) * box.height);
    if (aspect >= 0.55 && aspect <= 1.7 && fill >= 0.55) {
      best_area = area;
      best = box;
    }
  }
  return best;
}

class BlockDetector : public rclcpp::Node
{
public:
  BlockDetector() : Node("block_detector")
  {
    const auto share = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("recognized_blocks"));
    const auto path = declare_parameter<std::string>("image_path",
      (share / "images" / "cubes.png").string());
    const auto frame = cv::imread(path, cv::IMREAD_COLOR);
    if (frame.empty()) {
      throw std::runtime_error("无法读取图片 " + path);
    }
    cv::cvtColor(frame, hsv_, cv::COLOR_BGR2HSV);
    image_.format = "jpeg";
    if (!cv::imencode(".jpg", frame, image_.data, {cv::IMWRITE_JPEG_QUALITY, 95})) {
      throw std::runtime_error("无法压缩图片 " + path);
    }
    image_.header.frame_id = "sample_camera";

    image_pub_ = create_publisher<CompressedImage>("/camera/compressed", 2);
    marker_pub_ = create_publisher<ImageMarker>("/camera/block_marker", 2);
    service_ = create_service<DetectBlock>("/detect_block",
      [this](const std::shared_ptr<DetectBlock::Request> request,
        std::shared_ptr<DetectBlock::Response> response) { on_detect(*request, *response); });
    timer_ = create_wall_timer(std::chrono::milliseconds(500), [this] { publish_frame(); });
    RCLCPP_INFO(get_logger(), "已加载 %s，等待 /detect_block 选色请求", path.c_str());
  }

private:
  void on_detect(const DetectBlock::Request & request, DetectBlock::Response & response)
  {
    const auto color = parse_color(request.color);
    if (!color) {
      response.message = "请选择 red、blue、green 或红色、蓝色、绿色";
      return;
    }
    const auto name = *color == Color::Red ? "red" : *color == Color::Blue ? "blue" : "green";
    box_ = detect(hsv_, *color);
    if (box_) {
      response.found = true;
      response.x = box_->x;
      response.y = box_->y;
      response.width = box_->width;
      response.height = box_->height;
      response.message = std::string("已识别 ") + name;
      RCLCPP_INFO(get_logger(), "%s，x=%d y=%d width=%d height=%d",
        response.message.c_str(), box_->x, box_->y, box_->width, box_->height);
    } else {
      response.message = std::string("未找到 ") + name + " 方块";
    }
    publish_frame();
  }

  void publish_frame()
  {
    image_.header.stamp = now();
    ImageMarker marker;
    marker.header = image_.header;
    marker.ns = "block";
    marker.id = 0;
    marker.type = ImageMarker::LINE_STRIP;
    marker.action = ImageMarker::REMOVE;
    marker.scale = 3.0;
    marker.outline_color.r = 1.0F;
    marker.outline_color.a = 1.0F;

    if (box_) {
      marker.action = ImageMarker::ADD;
      const int x = box_->x;
      const int y = box_->y;
      const int right = x + box_->width - 1;
      const int bottom = y + box_->height - 1;
      for (const auto & pixel : std::vector<cv::Point>{
        {x, y}, {right, y}, {right, bottom}, {x, bottom}, {x, y}})
      {
        geometry_msgs::msg::Point point;
        point.x = pixel.x;
        point.y = pixel.y;
        marker.points.push_back(point);
      }
    }
    image_pub_->publish(image_);
    marker_pub_->publish(marker);
  }

  cv::Mat hsv_;
  std::optional<cv::Rect> box_;
  CompressedImage image_;
  rclcpp::Publisher<CompressedImage>::SharedPtr image_pub_;
  rclcpp::Publisher<ImageMarker>::SharedPtr marker_pub_;
  rclcpp::Service<DetectBlock>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<BlockDetector>());
  } catch (const std::exception & error) {
    RCLCPP_ERROR(rclcpp::get_logger("block_detector"), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
