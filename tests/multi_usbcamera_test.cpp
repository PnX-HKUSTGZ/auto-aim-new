#include <opencv2/opencv.hpp>
#include <thread>

#include "io/usbcamera/usbcamera.hpp"
#include "tools/exiter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{left l         |        /dev/usbcam_left  | 左侧USB相机端口名称 }"
  "{right r        |        /dev/usbcam_right | 右侧USB相机端口名称 }"
  "{back b         |        /dev/usbcam_back  | 后侧USB相机端口名称 }"
  "{@config-path   | configs/sentry.yaml    | 位置参数，yaml配置文件路径 }"
  "{d display      |                        | 显示视频流       }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  tools::Exiter exiter;

  auto config_path = cli.get<std::string>(0);
  auto left_device_name = cli.get<std::string>("left");
  auto right_device_name = cli.get<std::string>("right");
  auto back_device_name = cli.get<std::string>("back");
  auto display = cli.has("display");

  io::USBCamera usbcam1(left_device_name, config_path);
  std::this_thread::sleep_for(500ms);
  io::USBCamera usbcam2(right_device_name, config_path);
  std::this_thread::sleep_for(500ms);
  io::USBCamera back_camera(back_device_name, config_path);

  cv::Mat img1, img2, img3;
  std::chrono::steady_clock::time_point timestamp;
  auto last_stamp = std::chrono::steady_clock::now();
  while (!exiter.exit()) {
    usbcam1.read(img1, timestamp);
    usbcam2.read(img2, timestamp);
    back_camera.read(img3, timestamp);

    auto dt = tools::delta_time(timestamp, last_stamp);
    last_stamp = timestamp;

    tools::logger()->info("{:.2f} fps", 1 / dt);

    if (!display) continue;
    cv::imshow("img1", img1);
    cv::imshow("img2", img2);
    cv::imshow("img3", img3);

    if (cv::waitKey(1) == 'q') break;
  }
}
