// Copyright 2019 Bold Hearts
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "v4l2_camera/v4l2_camera.hpp"
#include "v4l2_camera/rate_bound_status.hpp"

#include <diagnostic_updater/update_functions.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <diagnostic_updater/publisher.hpp>

#include <sstream>
#include <stdexcept>
#include <string>
#include <memory>
#include <tuple>
#include <utility>
#include <vector>
#include <algorithm>

#include "v4l2_camera/fourcc.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "v4l2_camera/v4l2_camera_device.hpp"

#ifdef ENABLE_CUDA
#include <cuda.h>
#include <nppi_color_conversion.h>
#endif

using namespace std::chrono_literals;

namespace v4l2_camera
{

static void diagnoseDeviceNodeExistence(diagnostic_updater::DiagnosticStatusWrapper &stat,
                                        const std::string device_name,
                                        bool node_exist,
                                        std::mutex& mtx)
{
  std::lock_guard<std::mutex> lock(mtx);
  if (node_exist) {
    stat.summaryf(
        diagnostic_msgs::msg::DiagnosticStatus::OK,
        "%s is available", device_name.c_str());
    stat.add("Device node existence", "OK");
  } else {
    stat.summaryf(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR,
        "%s is unavailable", device_name.c_str());
    stat.add("Device node existence", "ERROR");
  }
}

static void
diagnoseV4l2BufferFlag(diagnostic_updater::DiagnosticStatusWrapper &stat,
                       const bool &is_v4l2_buffer_flag_error_detected,
                       std::mutex& mtx)
{
  std::lock_guard<std::mutex> lock(mtx);
  if (is_v4l2_buffer_flag_error_detected) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                 "Data might have been corrupted");
    stat.add("Buffer flag status", "WARN");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 "Data is dequeued successfully");
    stat.add("Buffer flag status", "OK");
  }
}  // static void DiagnoseV4l2BufferFlag

static void diagnoseStreamLiveness(
    diagnostic_updater::DiagnosticStatusWrapper &stat,
    const std::optional<uint32_t> &sequence, uint32_t &max_sequence_value,
    const std::optional<timeval> &raw_timestamp, int64_t &max_timestamp_value,
    bool &is_frame_updated, std::mutex& mtx)
{
  std::lock_guard<std::mutex> lock(mtx);
  if (!sequence || !raw_timestamp) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                 "Failed to dequeue/enqueue v4l2_buffer");
    return;
  }

  int64_t raw_timestamp_value = static_cast<int64_t>(raw_timestamp.value().tv_sec) * 1e9
                                + static_cast<int64_t>(raw_timestamp.value().tv_usec) * 1e3;

  auto is_error_condition = [&is_frame_updated](auto& lhs, auto& rhs){
    bool compare_result;
    if (is_frame_updated) {
      // If frame is updated, LHS should be greater than RHS,
      // so error condition will be the logical NOT.
      compare_result = (lhs <= rhs);
    } else {
      // If frame is not updated yet, LHS and RHS value may be the same
      compare_result = (lhs < rhs);
    }
    return compare_result;
  };

  if (is_error_condition(sequence.value(), max_sequence_value)) {
    // sequence should increase monotonically
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                 "Invalid sequence value is detected");
    stat.add("Stream liveness", "ERROR");
  } else if (is_error_condition(raw_timestamp_value, max_timestamp_value)) {
    // raw timestamp should also increase monotonically
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                 "Invalid timestamp value is detected");
    stat.add("Stream liveness", "ERROR");
  } else {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK,
                 "Sequence and timestamp value are valid");
    stat.add("Stream liveness", "OK");
    max_sequence_value = sequence.value();
    max_timestamp_value = raw_timestamp_value;
  }

  stat.addf("Sequence", "%u", sequence);
  stat.addf("Raw timestamp", "%lu", raw_timestamp_value);

  // reset flag
  is_frame_updated = false;
}  // static void diagnoseStreamLiveness

V4L2Camera::V4L2Camera(rclcpp::NodeOptions const & options)
: rclcpp::Node{"v4l2_camera", options},
  canceled_{false}
{
  // Prepare publisher
  // This should happen before registering on_set_parameters_callback,
  // else transport plugins will fail to declare their parameters
  bool use_sensor_data_qos = declare_parameter("use_sensor_data_qos", false);
  publish_rate_ = declare_parameter("publish_rate", -1.0);
  if(std::abs(publish_rate_) < std::numeric_limits<double>::epsilon()){
    RCLCPP_WARN(get_logger(), "Invalid publish_rate = 0. Use default value -1 instead");
    publish_rate_ = -1.0;
  }
  if(publish_rate_ > 0){
    const auto publish_period = rclcpp::Rate(publish_rate_).period();
    image_pub_timer_ = this->create_wall_timer(publish_period, [this](){this->publish_next_frame_=true;});
    publish_next_frame_ = false;
  }
  else{
    publish_next_frame_ = true;
  }
  const auto qos = use_sensor_data_qos ? rclcpp::SensorDataQoS() : rclcpp::QoS(10);

  use_image_transport_ = declare_parameter("use_image_transport", true);

  if (use_image_transport_) {
    camera_transport_pub_ = image_transport::create_camera_publisher(this, "image_raw",
                                                                    qos.get_rmw_qos_profile());
  } else {
    image_pub_ = create_publisher<sensor_msgs::msg::Image>("image_raw", qos);
    info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", qos);
  }

  // Prepare camera
  auto device_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  device_descriptor.description = "Path to video device";
  device_descriptor.read_only = true;
  auto device = declare_parameter<std::string>("video_device", "/dev/video2", device_descriptor); // JHH // Laptop: video2, Cart: video0

  auto use_v4l2_buffer_timestamps_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  use_v4l2_buffer_timestamps_descriptor.description = "Use v4l2 buffer timestamps";
  auto use_v4l2_buffer_timestamps = declare_parameter<bool>("use_v4l2_buffer_timestamps", true, use_v4l2_buffer_timestamps_descriptor);

  auto timestamp_offset_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  timestamp_offset_descriptor.description = "Timestamp offset in nanoseconds";
  auto timestamp_offset = declare_parameter<int64_t>("timestamp_offset", 0, timestamp_offset_descriptor);
  rclcpp::Duration timestamp_offset_duration = rclcpp::Duration::from_nanoseconds(timestamp_offset);

  // Prepare diagnostics
  auto hardware_id = declare_parameter<std::string>("hardware_id", "");
  min_ok_rate_ = declare_parameter<double>("min_ok_rate", 9.0);
  max_ok_rate_ = declare_parameter<double>("max_ok_rate", 11.0);
  min_warn_rate_ = declare_parameter<double>("min_warn_rate", 8.0);
  max_warn_rate_ = declare_parameter<double>("max_warn_rate", 12.0);
  observed_frames_transition_ = declare_parameter<int>("observed_frames_transition", 3);

  diag_updater_ = std::make_shared<diagnostic_updater::Updater>(this);
  diag_updater_->setHardwareID(hardware_id.empty() ? "none" : hardware_id);
  diag_composer_ = std::make_shared<diagnostic_updater::CompositeDiagnosticTask>(
      hardware_id.empty() ? "_diagnostics" : hardware_id + "_diagnostics");

  camera_ = std::make_shared<V4l2CameraDevice>(device, use_v4l2_buffer_timestamps, timestamp_offset_duration);

  if (!camera_->open()) {
    device_node_existence_diag_ = std::make_shared<diagnostic_updater::FunctionDiagnosticTask>(
        "device node existence", std::bind(&diagnoseDeviceNodeExistence, std::placeholders::_1,
                                           device, false, std::ref(lock_)));

    diag_composer_->addTask(device_node_existence_diag_.get());
    this->diag_updater_->add(*diag_composer_);
    this->diag_updater_->force_update();
    return;
  } else {
    device_node_existence_diag_ = std::make_shared<diagnostic_updater::FunctionDiagnosticTask>(
        "device node existence", std::bind(&diagnoseDeviceNodeExistence, std::placeholders::_1,
                                           device, true, std::ref(lock_)));
    diag_composer_->addTask(device_node_existence_diag_.get());
  }

  cinfo_ = std::make_shared<camera_info_manager::CameraInfoManager>(this, camera_->getCameraName());
#ifdef ENABLE_CUDA
  src_dev_ = std::allocate_shared<GPUMemoryManager>(allocator_);
  dst_dev_ = std::allocate_shared<GPUMemoryManager>(allocator_);
#endif

  // Read parameters and set up callback
  createParameters();

  // Start the camera
  if (!camera_->start()) {
    return;
  }

  // Start capture thread
  capture_thread_ = std::thread{
    [this]() -> void {
      if (!time_per_frame_ || !min_ok_rate_ || !max_ok_rate_ || !min_warn_rate_ || !max_warn_rate_) {
        return;
      }

      // Setup diagnostics
      custom_diagnostic_tasks::RateBoundStatus rate_bound_status(
          this,
          custom_diagnostic_tasks::RateBoundStatusParam(min_ok_rate_.value(), max_ok_rate_.value()),
          custom_diagnostic_tasks::RateBoundStatusParam(min_warn_rate_.value(), max_warn_rate_.value()),
          static_cast<size_t>(observed_frames_transition_), true,
          "rate bound check");
      diag_composer_->addTask(&rate_bound_status);

      double target_frequency = publish_rate_;
      if (target_frequency < 0) {
        if (std::abs(time_per_frame_.value()[1]) < std::numeric_limits<double>::epsilon() * 1e2) {
          // time_per_frame_ may be [0, 0] by default in some environments
          // In that case, diagnostics will be published at a rate between the min and max OK rate values
          target_frequency = (min_ok_rate_.value() + max_ok_rate_.value()) / 2;
        } else {
          target_frequency =
              static_cast<double>(time_per_frame_.value()[1]) / time_per_frame_.value()[0];
        }
      }
      diag_updater_->setPeriod(1./target_frequency);  // align diag rate and ideal topic rate

      bool is_v4l2_buffer_flag_error_detected = true;
      diagnostic_updater::FunctionDiagnosticTask buffer_flag_check_diag(
          "buffer flag check", std::bind(&diagnoseV4l2BufferFlag, std::placeholders::_1,
                                         std::cref(is_v4l2_buffer_flag_error_detected),
                                         std::ref(lock_)));

      diag_composer_->addTask(&buffer_flag_check_diag);

      std::optional<uint32_t> sequence = 0;
      uint32_t max_sequence_value = 0;
      std::optional<timeval> raw_timestamp{};
      int64_t max_timestamp_value = 0;
      bool is_frame_updated = false;
      diagnostic_updater::FunctionDiagnosticTask stream_liveness_check_diag(
          "stream liveness check", std::bind(&diagnoseStreamLiveness, std::placeholders::_1,
                                             std::cref(sequence), std::ref(max_sequence_value),
                                             std::cref(raw_timestamp), std::ref(max_timestamp_value),
                                             std::ref(is_frame_updated), std::ref(lock_)));

      diag_composer_->addTask(&stream_liveness_check_diag);
      this->diag_updater_->add(*diag_composer_);

      // Start capture loop
      while (rclcpp::ok() && !canceled_.load()) {
        RCLCPP_DEBUG(get_logger(), "Capture...");
        sensor_msgs::msg::Image::UniquePtr img;
        {
          std::lock_guard<std::mutex> lock(lock_);
          std::tie(img, is_v4l2_buffer_flag_error_detected, sequence, raw_timestamp) = camera_->capture();
          is_frame_updated = true;
        }
        if (img == nullptr) {
          // Failed capturing image, assume it is temporarily and continue a bit later
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          continue;
        }
        if(publish_next_frame_ == false){
          continue;
        }

        auto stamp = img->header.stamp;
        if (img->encoding != output_encoding_) {
#ifdef ENABLE_CUDA
          img = convertOnGpu(*img);
#else
          img = convert(*img);
#endif
        }
        img->header.stamp = stamp;
        img->header.frame_id = camera_frame_id_;

        auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(cinfo_->getCameraInfo());
        if (!checkCameraInfo(*img, *ci)) {
          *ci = sensor_msgs::msg::CameraInfo{};
          ci->height = img->height;
          ci->width = img->width;
        }

        ci->header.stamp = stamp;
        ci->header.frame_id = camera_frame_id_;
        publish_next_frame_ = publish_rate_ < 0;

        if (use_image_transport_) {
          camera_transport_pub_.publish(*img, *ci);
        } else {
          image_pub_->publish(std::move(img));
          info_pub_->publish(std::move(ci));
        }
        // Record the current timestamp to monitor the interval diagnostics
        rate_bound_status.tick();
      }
    }
  };
}

V4L2Camera::~V4L2Camera()
{
  canceled_.store(true);
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
}

void V4L2Camera::createParameters()
{
  // Node parameters
  auto output_encoding_description = rcl_interfaces::msg::ParameterDescriptor{};
  output_encoding_description.description = "ROS image encoding to use for the output image";
  output_encoding_description.additional_constraints =
    "Currently supported: 'rgb8', 'yuv422' or 'mono'";
  output_encoding_ = declare_parameter(
    "output_encoding", std::string{"rgb8"},
    output_encoding_description);

  // Camera info parameters
  auto camera_info_url = declare_parameter("camera_info_url", "");
  if (get_parameter("camera_info_url", camera_info_url)) {
    if (cinfo_->validateURL(camera_info_url)) {
      cinfo_->loadCameraInfo(camera_info_url);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
    }
  }

  auto camera_frame_id_description = rcl_interfaces::msg::ParameterDescriptor{};
  camera_frame_id_description.description = "Frame id inserted in published image";
  camera_frame_id_description.read_only = true;
  camera_frame_id_ = declare_parameter<std::string>(
    "camera_frame_id", "camera",
    camera_frame_id_description);

  // Format parameters
  // Pixel format
  auto const & image_formats = camera_->getImageFormats();
  auto pixel_format_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  pixel_format_descriptor.name = "pixel_format";
  pixel_format_descriptor.description = "Pixel format (FourCC)";
  auto pixel_format_constraints = std::ostringstream{};
  for (auto const & format : image_formats) {
    pixel_format_constraints <<
      "\"" << FourCC::toString(format.pixelFormat) << "\"" <<
      " (" << format.description << "), ";
  }
  auto str = pixel_format_constraints.str();
  str = str.substr(0, str.size() - 2);
  pixel_format_descriptor.additional_constraints = str;
  auto pixel_format =
    declare_parameter<std::string>("pixel_format", "YUYV", pixel_format_descriptor);
  requestPixelFormat(pixel_format);

  // Image size
  auto image_size = ImageSize{};
  auto image_size_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  image_size_descriptor.name = "image_size";
  image_size_descriptor.description = "Image width & height";

  // List available image sizes per format
  auto const & image_sizes = camera_->getImageSizes();
  auto image_sizes_constraints = std::ostringstream{};
  image_sizes_constraints << "Available image sizes:";

  for (auto const & format : image_formats) {
    image_sizes_constraints << "\n" << FourCC::toString(format.pixelFormat) << " (" <<
      format.description << ")";

    auto iter = image_sizes.find(format.pixelFormat);
    if (iter == image_sizes.end()) {
      RCLCPP_ERROR_STREAM(
        get_logger(),
        "No sizes available to create parameter description for format: " << format.description);
      continue;
    }

    auto size_type = iter->second.first;
    auto & sizes = iter->second.second;
    switch (size_type) {
      case V4l2CameraDevice::ImageSizeType::DISCRETE:
        for (auto const & image_size : sizes) {
          image_sizes_constraints << "\n\t" << image_size.first << "x" << image_size.second;
        }
        break;
      case V4l2CameraDevice::ImageSizeType::STEPWISE:
        image_sizes_constraints << "\n\tmin:\t" << sizes[0].first << "x" << sizes[0].second;
        image_sizes_constraints << "\n\tmax:\t" << sizes[1].first << "x" << sizes[1].second;
        image_sizes_constraints << "\n\tstep:\t" << sizes[2].first << "x" << sizes[2].second;
        break;
      case V4l2CameraDevice::ImageSizeType::CONTINUOUS:
        image_sizes_constraints << "\n\tmin:\t" << sizes[0].first << "x" << sizes[0].second;
        image_sizes_constraints << "\n\tmax:\t" << sizes[1].first << "x" << sizes[1].second;
        break;
    }
  }

  image_size_descriptor.additional_constraints = image_sizes_constraints.str();
  image_size = declare_parameter<ImageSize>("image_size", {640, 480}, image_size_descriptor);
  requestImageSize(image_size);

  // Time per frame
  auto tpf = camera_->getCurrentTimePerFrame();
  auto time_per_frame_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  time_per_frame_descriptor.name = "time_per_frame";
  time_per_frame_descriptor.description = "Desired period between successive frames in seconds";
  time_per_frame_descriptor.additional_constraints =
      "Length 2 array, with numerator (second) and denominator (frames)";
  time_per_frame_ = declare_parameter<TimePerFrame>(
      "time_per_frame", {tpf.first, tpf.second},
      time_per_frame_descriptor);
  if (camera_->timePerFrameSupported()) {
    requestTimePerFrame(time_per_frame_.value());
  }

  // Control parameters
  auto toParamName =
    [](std::string name) {
      std::transform(name.begin(), name.end(), name.begin(), ::tolower);
      name.erase(std::remove(name.begin(), name.end(), ','), name.end());
      name.erase(std::remove(name.begin(), name.end(), '('), name.end());
      name.erase(std::remove(name.begin(), name.end(), ')'), name.end());
      std::replace(name.begin(), name.end(), ' ', '_');
      return name;
    };

  for (auto const & c : camera_->getControls()) {
    auto name = toParamName(c.name);
    auto descriptor = rcl_interfaces::msg::ParameterDescriptor{};
    descriptor.name = name;
    descriptor.description = c.name;
    switch (c.type) {
      case ControlType::INT:
        {
          auto current_value = camera_->getControlValue(c.id);
          auto range = rcl_interfaces::msg::IntegerRange{};
          range.from_value = c.minimum;
          range.to_value = c.maximum;
          descriptor.integer_range.push_back(range);
          if (current_value < c.minimum || c.maximum < current_value) {
            current_value = c.defaultValue;
          }
          auto value = declare_parameter<int64_t>(name, current_value, descriptor);
          camera_->setControlValue(c.id, value);
          break;
        }
      case ControlType::BOOL:
        {
          auto value =
            declare_parameter<bool>(name, camera_->getControlValue(c.id) != 0, descriptor);
          camera_->setControlValue(c.id, value);
          break;
        }
      case ControlType::MENU:
        {
          auto sstr = std::ostringstream{};
          for (auto const & o : c.menuItems) {
            sstr << o.first << " - " << o.second << ", ";
          }
          auto str = sstr.str();
          descriptor.additional_constraints = str.substr(0, str.size() - 2);
          auto value = declare_parameter<int64_t>(name, camera_->getControlValue(c.id), descriptor);
          camera_->setControlValue(c.id, value);
          break;
        }
      default:
        RCLCPP_WARN(
          get_logger(),
          "Control type not currently supported: %s, for control: %s",
          std::to_string(unsigned(c.type)).c_str(),
          c.name.c_str());
        continue;
    }
    control_name_to_id_[name] = c.id;
  }

  // Register callback for parameter value setting
  on_set_parameters_callback_ = add_on_set_parameters_callback(
    [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult {
      auto result = rcl_interfaces::msg::SetParametersResult();
      result.successful = true;
      for (auto const & p : parameters) {
        result.successful &= handleParameter(p);
      }
      return result;
    });
}

bool V4L2Camera::handleParameter(rclcpp::Parameter const & param)
{
  auto name = std::string{param.get_name()};
  if (control_name_to_id_.find(name) != control_name_to_id_.end()) {
    switch (param.get_type()) {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        return camera_->setControlValue(control_name_to_id_[name], param.as_bool());
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        return camera_->setControlValue(control_name_to_id_[name], param.as_int());
      default:
        RCLCPP_WARN(
          get_logger(),
          "Control parameter type not currently supported: %s, for parameter: %s",
          std::to_string(unsigned(param.get_type())).c_str(), param.get_name().c_str());
    }
  } else if (param.get_name() == "output_encoding") {
    output_encoding_ = param.as_string();
    return true;
  } else if (param.get_name() == "pixel_format") {
    camera_->stop();
    auto success = requestPixelFormat(param.as_string());
    camera_->start();
    return success;
  } else if (param.get_name() == "image_size") {
    camera_->stop();
    auto success = requestImageSize(param.as_integer_array());
    camera_->start();
    return success;
  } else if (param.get_name() == "time_per_frame") {
    camera_->stop();
    auto success = requestTimePerFrame(param.as_integer_array());
    camera_->start();
    return success;
  } else if (param.get_name() == "camera_info_url") {
    auto camera_info_url = param.as_string();
    if (cinfo_->validateURL(camera_info_url)) {
      return cinfo_->loadCameraInfo(camera_info_url);
    } else {
      RCLCPP_WARN(get_logger(), "Invalid camera info URL: %s", camera_info_url.c_str());
      return false;
    }
  }

  return false;
}

bool V4L2Camera::requestPixelFormat(std::string const & fourcc)
{
  if (fourcc.size() != 4) {
    RCLCPP_ERROR(get_logger(), "Invalid pixel format size: must be a 4 character code (FOURCC).");
    return false;
  }

  auto code = v4l2_fourcc(fourcc[0], fourcc[1], fourcc[2], fourcc[3]);

  auto dataFormat = camera_->getCurrentDataFormat();
  // Do not apply if camera already runs at given pixel format
  if (dataFormat.pixelFormat == code) {
    return true;
  }

  dataFormat.pixelFormat = code;
  return camera_->requestDataFormat(dataFormat);
}

bool V4L2Camera::requestImageSize(std::vector<int64_t> const & size)
{
  if (size.size() != 2) {
    RCLCPP_WARN(
      get_logger(), "Invalid image size; expected dimensions: 2, actual: %lu", size.size());
    return false;
  }

  auto dataFormat = camera_->getCurrentDataFormat();
  // Do not apply if camera already runs at given size
  if (dataFormat.width == size[0] && dataFormat.height == size[1]) {
    return true;
  }

  dataFormat.width = size[0];
  dataFormat.height = size[1];
  return camera_->requestDataFormat(dataFormat);
}

bool V4L2Camera::requestTimePerFrame(TimePerFrame const & tpf)
{
  if (tpf.size() != 2) {
    RCLCPP_WARN(
      get_logger(), "Invalid time per frame; expected dimensions: 2, actual: %lu", tpf.size());
    return false;
  }

  return camera_->requestTimePerFrame(std::make_pair(tpf[0], tpf[1]));
}

static unsigned char CLIPVALUE(int val)
{
  // Old method (if)
  val = val < 0 ? 0 : val;
  return val > 255 ? 255 : val;
}

/**
 * Conversion from YUV to RGB.
 * The normal conversion matrix is due to Julien (surname unknown):
 *
 * [ R ]   [  1.0   0.0     1.403 ] [ Y ]
 * [ G ] = [  1.0  -0.344  -0.714 ] [ U ]
 * [ B ]   [  1.0   1.770   0.0   ] [ V ]
 *
 * and the firewire one is similar:
 *
 * [ R ]   [  1.0   0.0     0.700 ] [ Y ]
 * [ G ] = [  1.0  -0.198  -0.291 ] [ U ]
 * [ B ]   [  1.0   1.015   0.0   ] [ V ]
 *
 * Corrected by BJT (coriander's transforms RGB->YUV and YUV->RGB
 *                   do not get you back to the same RGB!)
 * [ R ]   [  1.0   0.0     1.136 ] [ Y ]
 * [ G ] = [  1.0  -0.396  -0.578 ] [ U ]
 * [ B ]   [  1.0   2.041   0.002 ] [ V ]
 *
 */
static void YUV2RGB(
  const unsigned char y, const unsigned char u, const unsigned char v, unsigned char * r,
  unsigned char * g, unsigned char * b)
{
  const int y2 = static_cast<int>(y);
  const int u2 = static_cast<int>(u) - 128;
  const int v2 = static_cast<int>(v) - 128;
  // std::cerr << "YUV=("<<y2<<","<<u2<<","<<v2<<")"<<std::endl;

  // This is the normal YUV conversion, but
  // appears to be incorrect for the firewire cameras
  //   int r2 = y2 + ( (v2*91947) >> 16);
  //   int g2 = y2 - ( ((u2*22544) + (v2*46793)) >> 16 );
  //   int b2 = y2 + ( (u2*115999) >> 16);
  // This is an adjusted version (UV spread out a bit)
  int r2 = y2 + ((v2 * 37221) >> 15);
  int g2 = y2 - (((u2 * 12975) + (v2 * 18949)) >> 15);
  int b2 = y2 + ((u2 * 66883) >> 15);
  // std::cerr << "   RGB=("<<r2<<","<<g2<<","<<b2<<")"<<std::endl;

  // Cap the values.
  *r = CLIPVALUE(r2);
  *g = CLIPVALUE(g2);
  *b = CLIPVALUE(b2);
}

static void yuyv2rgb(unsigned char const * YUV, unsigned char * RGB, int NumPixels)
{
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;

  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6) {
    y0 = YUV[i + 0];
    u = YUV[i + 1];
    y1 = YUV[i + 2];
    v = YUV[i + 3];
    YUV2RGB(y0, u, v, &r, &g, &b);
    RGB[j + 0] = r;
    RGB[j + 1] = g;
    RGB[j + 2] = b;
    YUV2RGB(y1, u, v, &r, &g, &b);
    RGB[j + 3] = r;
    RGB[j + 4] = g;
    RGB[j + 5] = b;
  }
}

static void uyvy2rgb(unsigned char const * YUV, unsigned char * RGB, int NumPixels)
{
  int i, j;
  unsigned char y0, y1, u, v;
  unsigned char r, g, b;

  for (i = 0, j = 0; i < (NumPixels << 1); i += 4, j += 6) {
    u = YUV[i + 0];
    y0 = YUV[i + 1];
    v = YUV[i + 2];
    y1 = YUV[i + 3];
    YUV2RGB(y0, u, v, &r, &g, &b);
    RGB[j + 0] = r;
    RGB[j + 1] = g;
    RGB[j + 2] = b;
    YUV2RGB(y1, u, v, &r, &g, &b);
    RGB[j + 3] = r;
    RGB[j + 4] = g;
    RGB[j + 5] = b;
  }
}

sensor_msgs::msg::Image::UniquePtr V4L2Camera::convert(sensor_msgs::msg::Image const & img) const
{
  RCLCPP_DEBUG(
    get_logger(),
    "Converting: %s -> %s", img.encoding.c_str(), output_encoding_.c_str());

  // TODO(sander): temporary until cv_bridge and image_proc are available in ROS 2
  if (img.encoding == sensor_msgs::image_encodings::YUV422_YUY2 &&
    output_encoding_ == sensor_msgs::image_encodings::RGB8)
  {
    auto outImg = std::make_unique<sensor_msgs::msg::Image>();
    outImg->width = img.width;
    outImg->height = img.height;
    outImg->step = img.width * 3;
    outImg->encoding = output_encoding_;
    outImg->data.resize(outImg->height * outImg->step);
    for (auto i = 0u; i < outImg->height; ++i) {
      yuyv2rgb(
        img.data.data() + i * img.step, outImg->data.data() + i * outImg->step,
        outImg->width);
    }
    return outImg;
  } else if (img.encoding == sensor_msgs::image_encodings::YUV422 &&
    output_encoding_ == sensor_msgs::image_encodings::RGB8)
  {
    auto outImg = std::make_unique<sensor_msgs::msg::Image>();
    outImg->width = img.width;
    outImg->height = img.height;
    outImg->step = img.width * 3;
    outImg->encoding = output_encoding_;
    outImg->data.resize(outImg->height * outImg->step);
    for (auto i = 0u; i < outImg->height; ++i) {
      uyvy2rgb(
        img.data.data() + i * img.step, outImg->data.data() + i * outImg->step,
        outImg->width);
    }
    return outImg;
  }else {
    RCLCPP_WARN_ONCE(
      get_logger(),
      "Conversion not supported yet: %s -> %s", img.encoding.c_str(), output_encoding_.c_str());
    return nullptr;
  }
}

bool V4L2Camera::checkCameraInfo(
  sensor_msgs::msg::Image const & img,
  sensor_msgs::msg::CameraInfo const & ci)
{
  return ci.width == img.width && ci.height == img.height;
}

#ifdef ENABLE_CUDA
sensor_msgs::msg::Image::UniquePtr V4L2Camera::convertOnGpu(sensor_msgs::msg::Image const & img)
{
  if ((img.encoding != sensor_msgs::image_encodings::YUV422 &&
       img.encoding != sensor_msgs::image_encodings::YUV422_YUY2) ||
      output_encoding_ != sensor_msgs::image_encodings::RGB8) {
    RCLCPP_WARN_ONCE(
        get_logger(),
        "Conversion not supported yet: %s -> %s", img.encoding.c_str(), output_encoding_.c_str());
    return nullptr;
  }

  auto outImg = std::make_unique<sensor_msgs::msg::Image>();
  outImg->width = img.width;
  outImg->height = img.height;
  outImg->step = img.width * 3;
  outImg->encoding = output_encoding_;
  outImg->data.resize(outImg->height * outImg->step);

  src_dev_->Allocate(img.width, img.height);
  dst_dev_->Allocate(outImg->width, outImg->height);

  unsigned int src_num_channel = static_cast<int>(img.step / img.width);  // No padded input is assumed
  cudaErrorCheck(cudaMemcpy2DAsync(static_cast<void*>(src_dev_->dev_ptr),
                                   src_dev_->step_bytes,
                                   static_cast<const void*>(img.data.data()),
                                   img.step,  // in byte. including padding
                                   img.width * src_num_channel * sizeof(Npp8u),  // in byte
                                   img.height,                 // in pixel
                                   cudaMemcpyHostToDevice));

  NppiSize roi = {static_cast<int>(img.width), static_cast<int>(img.height)};
  NppStatus res;
  if (img.encoding == sensor_msgs::image_encodings::YUV422_YUY2) {
    res = nppiYUV422ToRGB_8u_C2C3R(src_dev_->dev_ptr,
                                             src_dev_->step_bytes,
                                             dst_dev_->dev_ptr,
                                             dst_dev_->step_bytes,
                                             roi);
  } else {
    res = nppiCbYCr422ToRGB_8u_C2C3R(src_dev_->dev_ptr,
                                     src_dev_->step_bytes,
                                     dst_dev_->dev_ptr,
                                     dst_dev_->step_bytes,
                                     roi);
  }
  if (res != NPP_SUCCESS) {
    throw std::runtime_error{"NPPI operation failed"};
  }

  cudaErrorCheck(cudaMemcpy2DAsync(static_cast<void*>(outImg->data.data()),
                                   outImg->step,
                                   static_cast<const void*>(dst_dev_->dev_ptr),
                                   dst_dev_->step_bytes,
                                   outImg->width * 3 * sizeof(Npp8u),  // in byte. exclude padding
                                   outImg->height,
                                   cudaMemcpyDeviceToHost));

  cudaErrorCheck(cudaDeviceSynchronize());

  return outImg;
}
#endif
}  // namespace v4l2_camera

RCLCPP_COMPONENTS_REGISTER_NODE(v4l2_camera::V4L2Camera)
