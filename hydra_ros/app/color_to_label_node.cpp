/* -----------------------------------------------------------------------------
 * Copyright 2022 Massachusetts Institute of Technology.
 * All Rights Reserved
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Research was sponsored by the United States Air Force Research Laboratory and
 * the United States Air Force Artificial Intelligence Accelerator and was
 * accomplished under Cooperative Agreement Number FA8750-19-2-1000. The views
 * and conclusions contained in this document are those of the authors and should
 * not be interpreted as representing the official policies, either expressed or
 * implied, of the United States Air Force or the U.S. Government. The U.S.
 * Government is authorized to reproduce and distribute reprints for Government
 * purposes notwithstanding any copyright notation herein.
 * -------------------------------------------------------------------------- */
#include <hydra/common/semantic_color_map.h>
#include <ianvs/spin_functions.h>

#include <filesystem>

#include <CLI/CLI.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <cv_bridge/cv_mat_sensor_msgs_image_type_adapter.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hydra {

using sensor_msgs::msg::Image;
using ImageAdapter = rclcpp::TypeAdapter<cv_bridge::ROSCvMatContainer, Image>;

struct ColorToLabelNode : public rclcpp::Node {
  explicit ColorToLabelNode(const std::filesystem::path& colormap_path,
                            size_t queue_size = 1);

  void callback(const std_msgs::msg::Header& header, const cv::Mat& msg);

  int32_t default_label;
  std::unique_ptr<SemanticColorMap> colormap;
  rclcpp::Publisher<Image>::SharedPtr pub;
  rclcpp::Subscription<ImageAdapter>::SharedPtr sub;
};

ColorToLabelNode::ColorToLabelNode(const std::filesystem::path& colormap_path,
                                   size_t queue_size)
    : Node("color_to_label_node"),
      default_label(-1),
      colormap(SemanticColorMap::fromCsv(colormap_path)),
      pub(create_publisher<Image>("labels", queue_size)) {
  const auto cb = [this](const cv_bridge::ROSCvMatContainer& container) {
    callback(container.header(), container.cv_mat());
  };

  sub = create_subscription<ImageAdapter>("colors", queue_size, cb);
}

void ColorToLabelNode::callback(const std_msgs::msg::Header& header,
                                const cv::Mat& img) {
  if (!colormap) {
    RCLCPP_ERROR_STREAM(get_logger(), "Colormap is required!");
    return;
  }

  if (img.empty() || img.channels() != 3) {
    RCLCPP_ERROR_STREAM(get_logger(), "Failed to decode color image to semantics!");
    return;
  }

  cv::Mat labels(img.size(), CV_32SC1);
  for (int r = 0; r < img.rows; ++r) {
    for (int c = 0; c < img.cols; ++c) {
      const auto& pixel = img.at<cv::Vec3b>(r, c);
      const spark_dsg::Color color(pixel[0], pixel[1], pixel[2]);
      labels.at<int32_t>(r, c) =
          colormap->getLabelFromColor(color).value_or(default_label);
    }
  }

  cv_bridge::CvImage to_pub(header, "32SC1", labels);
  const auto msg = to_pub.toImageMsg();
  pub->publish(*msg);
}

}  // namespace hydra

struct AppArgs {
  void add_to_app(CLI::App& app);

  std::filesystem::path colormap_path;
  int32_t default_label = -1;
};

void AppArgs::add_to_app(CLI::App& app) {
  app.add_option("colormap_path", colormap_path)
      ->required()
      ->check(CLI::ExistingPath)
      ->description("Colormap to use");
  app.add_option("-l,--default-label", default_label);
}

int main(int argc, char* argv[]) {
  CLI::App app("Utility node to convert labels stored by color to integer values");
  argv = app.ensure_utf8(argv);
  app.allow_extras();
  app.get_formatter()->column_width(50);

  AppArgs args;
  args.add_to_app(app);
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  rclcpp::init(argc, argv);
  {  // scope for node
    hydra::ColorToLabelNode remapper(args.colormap_path);
    ianvs::NodeHandle nh(remapper);
    ianvs::spinAndWait(nh);
  }

  rclcpp::shutdown();
  return 0;
}
