#include <array>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include "ament_index_cpp/get_package_prefix.hpp"

extern char ** environ;

namespace
{
using namespace std::chrono_literals;

struct Options
{
  std::string image_path;
  int port = 8765;
};

Options parse_options(int argc, char ** argv)
{
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string flag = argv[i];
    if (flag != "--image-path" && flag != "--port") {
      throw std::invalid_argument("未知参数 " + flag + "，用 --help 查看用法。");
    }
    if (++i == argc) {
      throw std::invalid_argument(flag + " 缺少参数值。");
    }
    const std::string value = argv[i];
    if (flag == "--image-path") {
      if (value.empty()) {
        throw std::invalid_argument("图片路径不能为空。");
      }
      options.image_path = std::filesystem::absolute(value).string();
    } else {
      const auto end = value.data() + value.size();
      const auto result = std::from_chars(value.data(), end, options.port);
      if (result.ec != std::errc{} || result.ptr != end || options.port < 1 || options.port > 65535) {
        throw std::invalid_argument("--port 必须是 1 到 65535 之间的端口号。");
      }
    }
  }
  return options;
}

std::string quote_ros_string(const std::string & value)
{
  // ROS 2 会按 YAML 解析 -p 指定的值。
  std::string quoted = "\"";
  for (const char c : value) {
    switch (c) {
      case '\n': quoted += "\\n"; break;
      case '\r': quoted += "\\r"; break;
      case '\t': quoted += "\\t"; break;
      case '\"':
      case '\\': quoted += '\\'; [[fallthrough]];
      default: quoted += c;
    }
  }
  return quoted + '"';
}

std::string executable(const std::string & package, const std::string & name)
{
  return (std::filesystem::path(ament_index_cpp::get_package_prefix(package)) /
         "lib" / package / name).string();
}

void check(int error, const std::string & operation)
{
  if (error != 0) {
    throw std::system_error(error, std::generic_category(), operation);
  }
}

volatile sig_atomic_t stop_requested = 0;

void request_stop(int)
{
  stop_requested = 1;
}

pid_t wait_for_child(pid_t pid, int * status, int options)
{
  pid_t result;
  do {
    result = waitpid(pid, status, options);
  } while (result == -1 && errno == EINTR);
  return result;
}

class Session
{
public:
  Session()
  {
    struct sigaction action {};
    action.sa_handler = request_stop;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, nullptr) == -1 ||
      sigaction(SIGTERM, &action, nullptr) == -1)
    {
      check(errno, "sigaction");
    }
  }

  ~Session()
  {
    for (const auto & child : children_) {
      if (child.pid > 0) {
        kill(child.pid, SIGTERM);
      }
    }
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    for (const auto & child : children_) {
      if (child.pid <= 0) {
        continue;
      }
      while (wait_for_child(child.pid, nullptr, WNOHANG) == 0) {
        if (std::chrono::steady_clock::now() >= deadline) {
          kill(child.pid, SIGKILL);
          wait_for_child(child.pid, nullptr, 0);
          break;
        }
        std::this_thread::sleep_for(25ms);
      }
    }
  }

  void start(std::size_t slot, std::vector<std::string> arguments)
  {
    std::vector<char *> argv;
    for (auto & argument : arguments) {
      argv.push_back(argument.data());
    }
    argv.push_back(nullptr);

    posix_spawnattr_t attributes;
    check(posix_spawnattr_init(&attributes), "posix_spawnattr_init");
    // 子进程使用独立进程组，由启动程序统一处理 Ctrl+C。
    int error = posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP);
    if (error == 0) {
      error = posix_spawnattr_setpgroup(&attributes, 0);
    }
    pid_t pid = 0;
    if (error == 0) {
      error = posix_spawn(&pid, argv.front(), nullptr, &attributes, argv.data(), environ);
    }
    posix_spawnattr_destroy(&attributes);
    check(error, std::string("启动 ") + children_[slot].name);
    children_[slot].pid = pid;
  }

  int wait()
  {
    while (!stop_requested) {
      for (auto & child : children_) {
        int status = 0;
        const auto pid = wait_for_child(child.pid, &status, WNOHANG);
        if (pid == child.pid) {
          child.pid = 0;
          std::cerr << child.name << " 已退出，正在停止演示。\n";
          return WIFEXITED(status) && WEXITSTATUS(status) != 0 ? WEXITSTATUS(status) : 1;
        }
        if (pid == -1) {
          check(errno, "waitpid");
        }
      }
      std::this_thread::sleep_for(50ms);
    }
    return 0;
  }

private:
  struct Child
  {
    const char * name;
    pid_t pid = 0;
  };

  std::array<Child, 2> children_{{{"detector"}, {"foxglove_bridge"}}};
};
}  // namespace

int main(int argc, char ** argv)
{
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "用法: ros2 run recognized_blocks bringup [--image-path 图片路径] [--port 8765]\n";
    return 0;
  }
  try {
    const auto options = parse_options(argc, argv);
    std::vector<std::string> detector{executable("recognized_blocks", "detector")};
    if (!options.image_path.empty()) {
      detector.insert(detector.end(),
        {"--ros-args", "-p", "image_path:=" + quote_ros_string(options.image_path)});
    }
    const std::string port = std::to_string(options.port);
    Session session;
    session.start(0, detector);
    session.start(1, {executable("foxglove_bridge", "foxglove_bridge"),
      "--ros-args", "-p", "address:=127.0.0.1", "-p", "port:=" + port});
    std::cout << "Foxglove 连接地址：ws://localhost:" << port << std::endl;
    return session.wait();
  } catch (const std::exception & error) {
    std::cerr << "启动失败：" << error.what() << '\n';
    return 1;
  }
}
