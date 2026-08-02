// C++17 design skeleton for an OpenArm v2 left-arm SocketCAN bridge.
//
// This is intentionally not a hardware driver. Wire encoding, CAN identifiers,
// motor identity, torque sign, zero offsets, and hardware limits must be supplied
// by a verified adapter/configuration derived from enactic/openarm_can and tested
// against the real arm before ProtocolAdapter::verified() may return true.

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

#ifdef __linux__
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace openarm_bridge {

using Clock = std::chrono::steady_clock;
using Microseconds = std::chrono::microseconds;
constexpr std::size_t kJointCount = 7;

// ASSUMED proposed host timing; this is not a measured hardware guarantee.
constexpr Microseconds kCycle{2000};
constexpr Microseconds kReceiveBudget{250};

enum class SafetyState {
  kDisconnected,
  kConnectedDisabled,
  kInitializing,
  kZeroingOrCalibration,
  kEnabledHold,
  kActiveControl,
  kFault,
  kEstop,
};

enum class FaultCode {
  kNone,
  kStaleFeedback,
  kMissingAcknowledgment,
  kCanBusError,
  kPositionLimit,
  kVelocityLimit,
  kTorqueLimit,
  kTemperatureLimit,
  kInvalidNumeric,
  kMissedDeadline,
  kMotorControllerReset,
  kProtocolOrIdentityMismatch,
  kHostShutdown,
  kPhysicalEstop,
};

struct JointLimits {
  double hard_position_min_rad{};
  double hard_position_max_rad{};
  double hard_velocity_rad_s{};
  double normal_torque_nm{};
  double normal_torque_rate_nm_s{};  // ASSUMED until drive/arm validation.
  double absolute_torque_nm{};
  double maximum_temperature_c{};  // MISSING until motor/drive evidence is added.
};

struct MotorCommand {
  std::uint32_t motor_index{};  // Logical index; never assume this is a CAN ID.
  std::uint64_t host_sequence{};
  Clock::time_point host_timestamp{};
  double position_rad{};      // Used only if the verified motor mode accepts it.
  double velocity_rad_s{};    // Used only if the verified motor mode accepts it.
  double kp{};                // Zero for a verified host-torque-only mode.
  double kd{};                // Zero for a verified host-torque-only mode.
  double feedforward_torque_nm{};
};

struct MotorFeedback {
  std::uint32_t motor_index{};
  double position_rad{};
  double velocity_rad_s{};
  double estimated_torque_nm{};
  double temperature_c{};
  std::uint32_t device_status{};
  std::optional<std::uint16_t> device_sequence;
  std::optional<std::uint64_t> acknowledged_host_sequence;
};

struct FeedbackCacheEntry {
  MotorFeedback value{};
  Clock::time_point received_at{};
  std::uint64_t host_receive_sequence{};
  std::uint32_t missed_cycles{};
  bool valid{false};
};

#ifdef __linux__
using NativeCanFrame = can_frame;
#else
struct NativeCanFrame {
  std::uint32_t can_id{};
  std::uint8_t can_dlc{};
  std::array<std::uint8_t, 8> data{};
};
#endif

class ProtocolAdapter {
 public:
  virtual ~ProtocolAdapter() = default;
  virtual bool verified() const = 0;
  virtual std::optional<NativeCanFrame> encode(const MotorCommand& command) = 0;
  virtual std::optional<MotorFeedback> decode(const NativeCanFrame& frame) = 0;
  virtual std::optional<NativeCanFrame> disable(std::size_t motor_index) = 0;
  virtual bool provides_command_acknowledgment() const = 0;
  virtual bool identity_matches(std::size_t motor_index,
                                const MotorFeedback& feedback) const = 0;
};

// Deliberately refuses activation. Replace only with a protocol implementation
// verified against commit-pinned openarm_can code, captured frames, and hardware.
class UnverifiedProtocol final : public ProtocolAdapter {
 public:
  bool verified() const override { return false; }
  std::optional<NativeCanFrame> encode(const MotorCommand&) override {
    return std::nullopt;
  }
  std::optional<MotorFeedback> decode(const NativeCanFrame&) override {
    return std::nullopt;
  }
  std::optional<NativeCanFrame> disable(std::size_t) override {
    return std::nullopt;
  }
  bool provides_command_acknowledgment() const override { return false; }
  bool identity_matches(std::size_t, const MotorFeedback&) const override {
    return false;
  }
};

class SocketCanTransport {
 public:
  ~SocketCanTransport() { close(); }

  bool open(const std::string& interface_name) {
#ifdef __linux__
    close();
    socket_ = ::socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);
    if (socket_ < 0) return false;
    ifreq request{};
    std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
    if (::ioctl(socket_, SIOCGIFINDEX, &request) < 0) {
      close();
      return false;
    }
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (::bind(socket_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) <
        0) {
      close();
      return false;
    }
    return true;
#else
    (void)interface_name;
    return false;
#endif
  }

  // Nonblocking drain bounded by both EAGAIN and an absolute monotonic deadline.
  template <typename FrameHandler>
  bool receive_until(Clock::time_point deadline, FrameHandler&& handler) {
#ifdef __linux__
    while (Clock::now() < deadline) {
      NativeCanFrame frame{};
      const auto count = ::read(socket_, &frame, sizeof(frame));
      if (count == static_cast<ssize_t>(sizeof(frame))) {
        handler(frame, Clock::now());
        continue;
      }
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) return true;
      return false;
    }
    return true;
#else
    (void)deadline;
    (void)handler;
    return false;
#endif
  }

  bool send(const NativeCanFrame& frame) {
#ifdef __linux__
    return ::write(socket_, &frame, sizeof(frame)) ==
           static_cast<ssize_t>(sizeof(frame));
#else
    (void)frame;
    return false;
#endif
  }

  void close() {
#ifdef __linux__
    if (socket_ >= 0) ::close(socket_);
    socket_ = -1;
#endif
  }

 private:
#ifdef __linux__
  int socket_{-1};
#endif
};

bool finite_feedback(const MotorFeedback& feedback) {
  return std::isfinite(feedback.position_rad) &&
         std::isfinite(feedback.velocity_rad_s) &&
         std::isfinite(feedback.estimated_torque_nm) &&
         std::isfinite(feedback.temperature_c);
}

bool transition_allowed(SafetyState from, SafetyState to) {
  if (to == SafetyState::kEstop) return true;
  if (to == SafetyState::kFault && from != SafetyState::kEstop) return true;
  switch (from) {
    case SafetyState::kDisconnected:
      return to == SafetyState::kConnectedDisabled;
    case SafetyState::kConnectedDisabled:
      return to == SafetyState::kDisconnected ||
             to == SafetyState::kInitializing;
    case SafetyState::kInitializing:
      return to == SafetyState::kZeroingOrCalibration ||
             to == SafetyState::kConnectedDisabled;
    case SafetyState::kZeroingOrCalibration:
      return to == SafetyState::kEnabledHold ||
             to == SafetyState::kConnectedDisabled;
    case SafetyState::kEnabledHold:
      return to == SafetyState::kActiveControl ||
             to == SafetyState::kConnectedDisabled;
    case SafetyState::kActiveControl:
      return to == SafetyState::kEnabledHold;
    case SafetyState::kFault:
      return to == SafetyState::kConnectedDisabled;
    case SafetyState::kEstop:
      return to == SafetyState::kDisconnected;
  }
  return false;
}

class ControlLoop {
 public:
  ControlLoop(SocketCanTransport& transport, ProtocolAdapter& protocol,
              std::array<JointLimits, kJointCount> limits,
              Microseconds feedback_timeout,
              Microseconds acknowledgment_timeout,
              std::uint32_t expected_enabled_status)
      : transport_(transport),
        protocol_(protocol),
        limits_(limits),
        feedback_timeout_(feedback_timeout),
        acknowledgment_timeout_(acknowledgment_timeout),
        expected_enabled_status_(expected_enabled_status) {}

  bool run() {
    if (!protocol_.verified()) {
      trip(FaultCode::kProtocolOrIdentityMismatch,
           "protocol adapter is not hardware-verified");
      return false;
    }
    auto next_cycle = Clock::now();
    while (!stop_requested_.load()) {
      next_cycle += kCycle;
      const auto cycle_start = Clock::now();

      if (!receive_feedback(cycle_start + kReceiveBudget)) break;
      if (stop_requested_.load()) break;
      if (!validate_feedback(cycle_start)) break;
      if (state_ == SafetyState::kActiveControl ||
          state_ == SafetyState::kEnabledHold) {
        const auto commands = calculate_and_saturate_commands(cycle_start);
        if (!commands || !send_commands(*commands)) break;
      }
      enqueue_log_record_nonblocking(cycle_start);  // Never block the loop.

      if (Clock::now() > next_cycle) {
        ++missed_deadlines_;
        trip(FaultCode::kMissedDeadline, "2 ms host deadline missed");
        break;
      }
      std::this_thread::sleep_until(next_cycle);  // steady_clock is monotonic.
    }
    controlled_shutdown();
    return state_ != SafetyState::kFault && state_ != SafetyState::kEstop;
  }

  void request_stop() { stop_requested_.store(true); }

 private:
  bool transition_to(SafetyState target) {
    if (!transition_allowed(state_, target)) {
      trip(FaultCode::kProtocolOrIdentityMismatch,
           "prohibited safety-state transition");
      return false;
    }
    state_ = target;
    return true;
  }

  bool receive_feedback(Clock::time_point deadline) {
    return transport_.receive_until(
        deadline, [this](const NativeCanFrame& frame, Clock::time_point received) {
          const auto decoded = protocol_.decode(frame);
          if (!decoded || decoded->motor_index >= kJointCount ||
              !finite_feedback(*decoded)) {
            trip(FaultCode::kInvalidNumeric, "invalid CAN feedback frame");
            return;
          }
          auto& cache = feedback_[decoded->motor_index];
          if (cache.valid && decoded->device_sequence &&
              cache.value.device_sequence &&
              *decoded->device_sequence == *cache.value.device_sequence) {
            ++cache.missed_cycles;  // Duplicate is not fresh feedback.
            return;
          }
          cache.value = *decoded;
          cache.received_at = received;
          cache.host_receive_sequence = ++receive_sequence_;
          cache.missed_cycles = 0;
          cache.valid = true;
          if (decoded->acknowledged_host_sequence &&
              *decoded->acknowledged_host_sequence >
                  last_acknowledged_sequence_[decoded->motor_index]) {
            last_acknowledged_sequence_[decoded->motor_index] =
                *decoded->acknowledged_host_sequence;
            last_acknowledgment_at_[decoded->motor_index] = received;
          }
        });
  }

  bool validate_feedback(Clock::time_point now) {
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      auto& entry = feedback_[joint];
      if (!entry.valid || now - entry.received_at > feedback_timeout_) {
        ++entry.missed_cycles;
        trip(FaultCode::kStaleFeedback, "missing or stale motor feedback");
        return false;
      }
      if (!protocol_.identity_matches(joint, entry.value)) {
        trip(FaultCode::kProtocolOrIdentityMismatch, "motor identity mismatch");
        return false;
      }
      const auto& value = entry.value;
      const auto& limit = limits_[joint];
      if (value.position_rad < limit.hard_position_min_rad ||
          value.position_rad > limit.hard_position_max_rad) {
        trip(FaultCode::kPositionLimit, "hard joint position exceeded");
        return false;
      }
      if (std::abs(value.velocity_rad_s) > limit.hard_velocity_rad_s) {
        trip(FaultCode::kVelocityLimit, "hard joint velocity exceeded");
        return false;
      }
      if (std::abs(value.estimated_torque_nm) > limit.absolute_torque_nm) {
        trip(FaultCode::kTorqueLimit, "absolute feedback torque exceeded");
        return false;
      }
      if (value.temperature_c > limit.maximum_temperature_c) {
        trip(FaultCode::kTemperatureLimit, "motor temperature exceeded");
        return false;
      }
      if (protocol_.provides_command_acknowledgment() &&
          last_command_sent_[joint] != Clock::time_point{} &&
          now - last_acknowledgment_at_[joint] > acknowledgment_timeout_) {
        trip(FaultCode::kMissingAcknowledgment,
             "command acknowledgment/feedback is stale");
        return false;
      }
      if (value.device_status != expected_enabled_status_) {
        trip(FaultCode::kMotorControllerReset,
             "motor status changed or controller reset");
        return false;
      }
    }
    return true;
  }

  std::optional<std::array<MotorCommand, kJointCount>>
  calculate_and_saturate_commands(Clock::time_point now) {
    std::array<MotorCommand, kJointCount> commands{};
    ++command_sequence_;
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      const auto desired = desired_state_for_cycle(joint, now);
      const auto measured = feedback_[joint].value;
      const double requested_torque = outer_loop_torque(joint, desired, measured);
      if (!std::isfinite(requested_torque)) {
        trip(FaultCode::kInvalidNumeric, "non-finite controller output");
        return std::nullopt;
      }
      if (std::abs(requested_torque) > limits_[joint].absolute_torque_nm) {
        trip(FaultCode::kTorqueLimit, "absolute requested torque exceeded");
        return std::nullopt;
      }
      const double amplitude_limited_torque =
          std::max(-limits_[joint].normal_torque_nm,
                   std::min(limits_[joint].normal_torque_nm, requested_torque));
      const double maximum_change =
          limits_[joint].normal_torque_rate_nm_s *
          std::chrono::duration<double>(kCycle).count();
      const double constrained_torque =
          std::max(last_constrained_torque_nm_[joint] - maximum_change,
                   std::min(last_constrained_torque_nm_[joint] + maximum_change,
                            amplitude_limited_torque));
      last_constrained_torque_nm_[joint] = constrained_torque;
      commands[joint] = MotorCommand{
          static_cast<std::uint32_t>(joint), command_sequence_, now,
          desired.position_rad, desired.velocity_rad_s,
          /*kp=*/0.0, /*kd=*/0.0, constrained_torque};
    }
    return commands;
  }

  bool send_commands(
      const std::array<MotorCommand, kJointCount>& commands) {
    for (const auto& command : commands) {
      const auto frame = protocol_.encode(command);
      if (!frame || !transport_.send(*frame)) {
        trip(FaultCode::kCanBusError, "CAN command transmission failed");
        return false;
      }
      last_command_sent_[command.motor_index] = Clock::now();
      if (last_acknowledgment_at_[command.motor_index] == Clock::time_point{}) {
        last_acknowledgment_at_[command.motor_index] =
            last_command_sent_[command.motor_index];
      }
    }
    return true;
  }

  struct DesiredState {
    double position_rad{};
    double velocity_rad_s{};
  };

  DesiredState desired_state_for_cycle(std::size_t, Clock::time_point) {
    // TODO: connect the validated seven-joint trajectory generator. A FAULT or
    // stop request freezes/stops trajectory advancement before this function.
    return {};
  }

  double outer_loop_torque(std::size_t, const DesiredState&,
                           const MotorFeedback&) {
    // TODO: port the tested host PD + hardware dynamics estimate. Simulation
    // qfrc_bias cannot be copied blindly to hardware.
    return 0.0;
  }

  void enqueue_log_record_nonblocking(Clock::time_point) {
    // TODO: push into a bounded lock-free/preallocated queue. On overflow,
    // increment a counter; never perform file I/O in the 2 ms control loop.
  }

  void trip(FaultCode code, const std::string& reason) {
    if (fault_ == FaultCode::kNone) {
      fault_ = code;
      fault_time_ = Clock::now();
      fault_reason_ = reason;
    }
    state_ = (code == FaultCode::kPhysicalEstop) ? SafetyState::kEstop
                                                 : SafetyState::kFault;
    stop_requested_.store(true);  // Stops trajectory and active control.
  }

  void controlled_shutdown() {
    // Declared default for unvalidated hardware: stop trajectory, send zero
    // torque/disable frames if communication is healthy, then close CAN. A
    // gravity-loaded arm may fall; brakes/support and a formal safe-state policy
    // are required before hardware use.
    for (std::size_t joint = 0; joint < kJointCount; ++joint) {
      if (const auto frame = protocol_.disable(joint)) transport_.send(*frame);
    }
    transport_.close();
    flush_logs_outside_realtime_loop();
  }

  void flush_logs_outside_realtime_loop() {
    // TODO: preserve state, fault code/reason/time, counters, and recent frames.
  }

  SocketCanTransport& transport_;
  ProtocolAdapter& protocol_;
  std::array<JointLimits, kJointCount> limits_{};
  std::array<FeedbackCacheEntry, kJointCount> feedback_{};
  std::array<Clock::time_point, kJointCount> last_command_sent_{};
  std::array<Clock::time_point, kJointCount> last_acknowledgment_at_{};
  std::array<std::uint64_t, kJointCount> last_acknowledged_sequence_{};
  std::array<double, kJointCount> last_constrained_torque_nm_{};
  Microseconds feedback_timeout_;
  Microseconds acknowledgment_timeout_;
  std::atomic<bool> stop_requested_{false};
  SafetyState state_{SafetyState::kDisconnected};
  FaultCode fault_{FaultCode::kNone};
  Clock::time_point fault_time_{};
  std::string fault_reason_;
  std::uint64_t receive_sequence_{};
  std::uint64_t command_sequence_{};
  std::uint64_t missed_deadlines_{};
  std::uint32_t expected_enabled_status_;  // Supplied by verified adapter.
};

}  // namespace openarm_bridge

int main() {
  std::cerr
      << "Design skeleton only: no verified OpenArm CAN protocol/configuration "
         "is linked; refusing to enable hardware.\n";
  return 2;
}
