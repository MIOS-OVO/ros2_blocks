#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "block_interfaces/srv/detect_block.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
using DetectBlock = block_interfaces::srv::DetectBlock;
using namespace std::chrono_literals;

int select_colors()
{
  const auto node = rclcpp::Node::make_shared("select_block_color");
  const auto client = node->create_client<DetectBlock>("/detect_block");
  if (!client->wait_for_service(5s)) {
    std::cerr << "未连接到 /detect_block，请先启动识别节点。\n";
    return 1;
  }

  while (rclcpp::ok()) {
    std::cout << "输入颜色（红/蓝/绿，q 退出）> " << std::flush;
    std::string color;
    if (!std::getline(std::cin, color) || !rclcpp::ok()) {
      break;
    }
    const auto first = color.find_first_not_of(" \t\r\n\v\f");
    if (first == std::string::npos) {
      continue;
    }
    color = color.substr(first, color.find_last_not_of(" \t\r\n\v\f") - first + 1);
    if (color == "q" || color == "Q") {
      break;
    }

    const auto request = std::make_shared<DetectBlock::Request>();
    request->color = color;
    auto pending = client->async_send_request(request);
    const auto status = rclcpp::spin_until_future_complete(node, pending, 5s);
    if (status != rclcpp::FutureReturnCode::SUCCESS) {
      client->remove_pending_request(pending);
      if (status == rclcpp::FutureReturnCode::INTERRUPTED) {
        break;
      }
      std::cerr << "服务响应超时，请检查识别节点。\n";
      continue;
    }
    const auto response = pending.get();
    std::cout << response->message << '\n';
    if (response->found) {
      std::cout << "x=" << response->x << "  y=" << response->y
                << "  width=" << response->width << "  height=" << response->height << '\n';
    }
  }
  return 0;
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const int status = select_colors();
  rclcpp::shutdown();
  return status;
}
