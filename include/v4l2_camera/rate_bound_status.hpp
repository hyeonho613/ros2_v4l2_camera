// Copyright 2025 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RATE_BOUND_STATUS_HPP_
#define RATE_BOUND_STATUS_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <iomanip>
#include <sstream>
#include <variant>
#include <optional>
#include <mutex>

namespace rate_bound_status
{
/**
 * \brief A structure that holds the constructor parameters for the
 * RateBoundStatus class.
 */
struct RateBoundStatusParam
{
  RateBoundStatusParam(const double min_freq, const double max_freq)
      : min_frequency(min_freq), max_frequency(max_freq){}

  double min_frequency;
  double max_frequency;
};

/**
 * \brief Diagnostic task to monitor the interval between events.
 *
 * This diagnostic task monitors the difference between consecutive events,
 * and creates corresponding diagnostics. This task categorize observed intervals into
 * OK/WARN/ERROR according to the value ranges passed via constructor arguments
 */
class RateBoundStatus : public diagnostic_updater::DiagnosticTask
{
private:
  // Helper struct to express state machine nodes
  struct StateBase
  {
    StateBase(const unsigned char lv, const std::string m)
        : level(lv), num_observation(0), msg(m) {}

    unsigned char level;
    size_t num_observation;
    std::string msg;
  };

  struct Stale : public StateBase
  {
    Stale()
        : StateBase(diagnostic_msgs::msg::DiagnosticStatus::STALE,
                    "Topic has not been received yet") {}
  };

  struct Ok : public StateBase
  {
    Ok()
        : StateBase(diagnostic_msgs::msg::DiagnosticStatus::OK,
                    "Rate is reasonable") {}
  };

  struct Warn : public StateBase
  {
    Warn()
        : StateBase(diagnostic_msgs::msg::DiagnosticStatus::WARN,
                    "Rate is within warning range") {}
  };

  struct Error : public StateBase
  {
    Error()
        : StateBase(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                    "Rate is out of valid range") {}
  };

  using StateHolder = std::variant<Stale, Ok, Warn, Error>;

 public:
  /**
   * \brief Constructs RateBoundstatus, which inherits diagnostic_updater::DiagnosticTask.
   *
   * \param ok_params The pair of min/max frequency for the topic rate to be recognized as "OK".
   * \param warn_params The pair of min/max frequency for the topic rate to be recognized as "WARN".
   * \param num_frame_transition The number of the successive observations for the status
   * transition. E.g., the status will not be changed from OK to WARN until successive
   * `num_frame_transition` WARNs are observed.
   * \param name The arbitral string to be assigned for this diagnostic task.
   * This name will not be exposed in the actual published topics.
   */
  RateBoundStatus(const RateBoundStatusParam& ok_params,
                  const RateBoundStatusParam& warn_params,
                  const size_t num_frame_transition = 1,
                  std::string name = "rate bound check")
      : DiagnosticTask(name), ok_params_(ok_params), warn_params_(warn_params),
        num_frame_transition_(num_frame_transition), zero_seen_(false),
        candidate_state_(Stale{}), current_state_(Stale{})
  {}

  /**
   * \brief Signals an event.
   *
   * \param t The timestamp of the event that will be used in computing
   * intervals.
   */
  void tick(double stamp)
  {
    std::unique_lock<std::mutex> lock(lock_);
    if (!previous_frame_timestamp_) {
      previous_frame_timestamp_ = rclcpp::Clock().now().seconds();
    }

    if (stamp == 0) {
      zero_seen_ = true;
    } else {
      zero_seen_ = false;
      double delta = stamp - previous_frame_timestamp_.value();
      frequency_ = (delta < 10*std::numeric_limits<double>::epsilon()) ?
                   std::numeric_limits<double>::infinity() : 1. / delta;
    }
    previous_frame_timestamp_ = stamp;
  }

  void tick(const rclcpp::Time& t) {tick(t.seconds());}

  /**
   * \brief function called every update
   */
  void run(diagnostic_updater::DiagnosticStatusWrapper& stat) override
  {
    std::unique_lock<std::mutex> lock(lock_);

    // classify the current observation
    StateHolder frame_result;
    if (!frequency_ || zero_seen_) {
      frame_result.emplace<Stale>();
    } else {
      if (ok_params_.min_frequency < frequency_ && frequency_ < ok_params_.max_frequency) {
        frame_result.emplace<Ok>();
      } else if ((warn_params_.min_frequency <= frequency_ && frequency_ <= ok_params_.min_frequency) ||
                 (ok_params_.max_frequency <= frequency_ && frequency_ <= warn_params_.max_frequency)) {
        frame_result.emplace<Warn>();
      } else {
        frame_result.emplace<Error>();
      }
    }

    // If the classify result is same as previous one, count the number of observation
    // Otherwise, update candidate
    if (candidate_state_.index() == frame_result.index()) {  // if result has the same status as candidate
      std::visit([](auto& s){
        s.num_observation += 1;
      }, candidate_state_);
    } else {
      candidate_state_ = frame_result;
    }

    // Update the current state if the same state is observed multiple times
    if (get_num_observation(candidate_state_) >= num_frame_transition_) {
      current_state_ = candidate_state_;
      std::visit([](auto& s) {
        s.num_observation = 0;
      }, candidate_state_);
    }

    stat.summary(get_level(current_state_), get_msg(current_state_));

    std::stringstream ss;
    ss << "observed=" << std::fixed << std::setprecision(2) << frequency_.value_or(0.0)
       << ", level=" << get_level_string(get_level(frame_result));
    stat.add("rate", ss.str());

    ss.str("");  // reset contents;
    ss << std::fixed << std::setprecision(2)
       << "OK(" << ok_params_.min_frequency << ", " << ok_params_.max_frequency << "), "
       << "WARN[" << warn_params_.min_frequency << ", " << warn_params_.max_frequency << "]";
    stat.add("rate criteria", ss.str());

    ss.str("");  // reset contents;
    ss << "level=" << get_level_string(get_level(candidate_state_))
       << " (num observation=" << get_num_observation(candidate_state_) << ")";
    stat.add("rate state next candidate", ss.str());
  }

protected:
  RateBoundStatusParam ok_params_;
  RateBoundStatusParam warn_params_;
  size_t num_frame_transition_;
  bool zero_seen_;
  std::optional<double> frequency_;
  std::optional<double> previous_frame_timestamp_;
  std::mutex lock_;

  StateHolder candidate_state_;
  StateHolder current_state_;

  unsigned char get_level(const StateHolder& state) {
    return std::visit([](const auto& s){return s.level;}, state);
  }

  size_t get_num_observation(const StateHolder& state) {
    return std::visit([](const auto& s){return s.num_observation;}, state);
  }

  std::string get_msg(const StateHolder& state) {
    return std::visit([](const auto& s){return s.msg;}, state);
  }

  std::string get_level_string(unsigned char level) {
    switch(level) {
      case diagnostic_msgs::msg::DiagnosticStatus::OK:
        return "OK";
      case diagnostic_msgs::msg::DiagnosticStatus::WARN:
        return "WARN";
      case diagnostic_msgs::msg::DiagnosticStatus::ERROR:
        return "ERROR";
      case diagnostic_msgs::msg::DiagnosticStatus::STALE:
        return "STALE";
      default:
        return "UNDEFINED";
    }
  }

};  // class RateBoundStatus

}  // namespace rate_bound_status

#endif  // RATE_BOUND_STATUS_HPP_
