#include "unitree_controller.hpp"

#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
constexpr size_t kMotors = 29;
constexpr size_t kArm = 15;
constexpr size_t kArms = 10;

void Check(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

template <typename Function>
void CheckRejected(Function&& fn, const char* message) {
    try {
        fn();
    } catch (const std::exception&) {
        return;
    }
    throw std::runtime_error(message);
}

UnitreeConfig Config(bool observer, bool active) {
    UnitreeConfig cfg{};
    cfg.msg_type = "hg";
    cfg.hand_type = "NONE";
    cfg.control_mode = ControlMode::POSITION;
    cfg.num_dofs = kMotors;
    cfg.command_timeout = cfg.state_timeout = 0.1;
    cfg.stiffness.assign(kMotors, 10);
    cfg.damping.assign(kMotors, 1);
    cfg.arm_effort_enabled = active;
    if (observer) {
        cfg.arm_motor_indices = {15, 16, 17, 18, 19, 22, 23, 24, 25, 26};
        cfg.arm_total_torque_limits.assign(kArms, 2);
        cfg.arm_effort_limits.assign(kArms, 0.5);
    }
    return cfg;
}
}  // namespace

struct UnitreeObserverTest {
    static double Feed(UnitreeController& ctrl, uint32_t tick) {
        LowState_ packet{};
        packet.tick() = tick;
        packet.crc() = ctrl.Crc32Core(reinterpret_cast<uint32_t*>(&packet), (sizeof(packet) >> 2) - 1);
        ctrl.LowStateHandler(&packet);
        return ctrl.get_robot_state().received_time_sec;
    }

    static void Run() {
        std::vector<LowCmd_> packets;
        bool fail_next = false;
        const auto sink = [&](const LowCmd_& packet) {
            if (fail_next) {
                fail_next = false;
                return false;
            }
            packets.push_back(packet);
            return true;
        };
        UnitreeController off(Config(false, false), sink);
        std::vector<double> target(kMotors, 0.12);
        std::vector<double> efforts(kArms, 0);
        off.step(target);
        Check(packets.back().motor_cmd()[kArm].tau() == 0 && packets.back().motor_cmd()[kArm].kp() == 10,
              "OFF must keep the normal position packet");

        UnitreeController shadow(Config(true, false), sink);
        auto stamp = Feed(shadow, 1);
        shadow.arm_position_control();
        auto receipt = shadow.step(target, efforts, shadow.get_safety_status().reset_generation);
        Check(receipt.arm_efforts[0] == 0 && receipt.command_time_sec >= stamp, "Invalid SHADOW receipt");
        Check(packets.back().motor_cmd()[kArm].tau() == 0, "SHADOW transmitted torque");
        Check(Feed(shadow, 1) == stamp, "Repeated tick refreshed the snapshot");

        auto torso_cfg = Config(true, false);
        Check(!torso_cfg.enable_torso_imu, "Torso IMU must be opt-in");
        torso_cfg.enable_torso_imu = true;
        UnitreeController torso(torso_cfg, sink);
        const auto first_stamp = Feed(torso, 1);
        unitree_hg::msg::dds_::IMUState_ torso_packet{};
        torso_packet.gyroscope()[1] = 0.25f;
        torso.TorsoImuStateHandler(&torso_packet);
        Check(Feed(torso, 1) == first_stamp, "Torso IMU refreshed a repeated LowState tick");
        const auto next_stamp = Feed(torso, 2);
        const auto torso_state = torso.get_robot_state();
        Check(next_stamp >= first_stamp && torso_state.tick == 2 &&
                  torso_state.torso_imu_state.gyroscope[1] == 0.25f && torso_state.imu_state.gyroscope[1] == 0,
              "Torso IMU did not stay separate from timestamped base feedback");

        UnitreeController ticking(Config(true, false), sink);
        const auto final_tick = std::numeric_limits<uint32_t>::max();
        Feed(ticking, final_tick);
        ticking.arm_position_control();
        Feed(ticking, 0);
        Check(!ticking.get_safety_status().latched, "A wrapping feedback tick latched damping");
        Feed(ticking, final_tick - 1);
        Check(ticking.get_safety_status().latched, "A backwards feedback tick did not latch damping");

        UnitreeController active(Config(true, true), sink);
        Feed(active, 1);
        active.arm_position_control();
        auto generation = active.get_safety_status().reset_generation;
        efforts[0] = 0.2;
        receipt = active.step(target, efforts, generation);
        const auto& motor = packets.back().motor_cmd()[kArm];
        Check(receipt.arm_efforts[0] <= efforts[0] && receipt.arm_efforts[0] > 0.19 &&
                  motor.tau() == receipt.arm_efforts[0] && motor.q() == static_cast<float>(target[kArm]),
              "Position+effort packet or rounding is wrong");
        target[kArm] = 0.19;
        efforts[0] = 1;
        receipt = active.step(target, efforts, generation);
        Check(receipt.arm_efforts[0] > 0.09 && receipt.arm_efforts[0] <= 0.101, "PD headroom was not enforced");
        target[kArm] = 0.12;
        receipt = active.step(target, efforts, generation);
        Check(receipt.arm_efforts[0] == 0.5, "Additive cap was not enforced");
        active.clear_arm_effort();
        Check(packets.back().motor_cmd()[kArm].tau() == 0, "Clearing did not publish zero torque");
        receipt = active.step(target, efforts, generation);
        Check(receipt.reset_generation != generation && receipt.arm_efforts[0] == 0 &&
                  packets.back().motor_cmd()[kArm].tau() == 0,
              "Old generation admitted effort");
        const std::vector<double> wrong_layout(kMotors, 1);
        generation = active.get_safety_status().reset_generation;
        CheckRejected([&] { active.step(target, wrong_layout, generation); },
                      "Motor-sized effort bypassed the arm layout");
        CheckRejected([&] { active.step(target); }, "Normal step rearmed latched damping");
        active.arm_position_control();
        generation = active.get_safety_status().reset_generation;
        fail_next = true;
        const auto packets_before_failure = packets.size();
        CheckRejected([&] { active.step(target, efforts, generation); }, "Failed publish acknowledged");
        Check(packets.size() == packets_before_failure + 1, "Failed publish duplicated the damping packet");
        Check(packets.back().motor_cmd()[kArm].tau() == 0 && packets.back().motor_cmd()[kArm].kp() == 0,
              "Publish failure did not clear effort in damping");
        active.set_passive();
        active.set_damping(1);
        Check(packets.back().motor_cmd()[kArm].kd() == 5, "Mode commands replaced latched damping");

        Feed(active, 2);
        active.arm_position_control();
        active.step(target, efforts, active.get_safety_status().reset_generation);
        active.last_command_time_ns_.store(active.MonotonicNowNs() - 1000000000);
        active.LowCommandWriter();
        Check(packets.back().motor_cmd()[kArm].tau() == 0, "Command watchdog retained torque");
        CheckRejected([&] { active.step(target); }, "Command watchdog was not latched");

        Feed(active, 3);
        active.arm_position_control();
        active.step(target, efforts, active.get_safety_status().reset_generation);
        active.last_state_time_ns_.store(active.MonotonicNowNs() - 1000000000);
        active.LowCommandWriter();
        Check(packets.back().motor_cmd()[kArm].tau() == 0, "Feedback watchdog retained torque");
        CheckRejected([&] { active.arm_position_control(); }, "Stale feedback allowed rearm");

        Feed(active, 4);
        active.arm_position_control();
        active.step(target, efforts, active.get_safety_status().reset_generation);
        active.set_gains(target, target);
        Check(packets.back().motor_cmd()[kArm].tau() == 0, "Gain change retained torque");
        auto oversized = target;
        oversized[0] = std::numeric_limits<double>::max();
        CheckRejected([&] { active.set_gains(oversized, target); }, "Gain outside motor float range accepted");
        CheckRejected([&] { active.step(oversized); }, "Position outside motor float range accepted");
        Check(active.get_safety_status().latched, "Invalid position did not latch damping");
        active.shutdown();
        CheckRejected([&] { active.arm_position_control(); }, "Shutdown allowed rearm");

        auto full_cfg = Config(true, true);
        full_cfg.arm_motor_indices = {28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16, 15};
        const auto arm_count = full_cfg.arm_motor_indices.size();
        full_cfg.arm_total_torque_limits.assign(arm_count, 2);
        full_cfg.arm_effort_limits.assign(arm_count, 0.5);
        UnitreeController full(full_cfg, sink);
        Feed(full, 1);
        full.arm_position_control();
        std::vector<double> full_efforts(arm_count);
        for (size_t i = 0; i < arm_count; ++i)
            full_efforts[i] = (i + 1) / 64.0;
        receipt = full.step(target, full_efforts, full.get_safety_status().reset_generation);
        for (size_t i = 0; i < arm_count; ++i) {
            const auto slot = full_cfg.arm_motor_indices[i];
            Check(receipt.arm_efforts[i] == full_efforts[i] &&
                      packets.back().motor_cmd()[slot].tau() == receipt.arm_efforts[i],
                  "Full G1 effort did not follow configured motor and receipt order");
        }
        for (size_t slot = 0; slot < 15; ++slot)
            Check(packets.back().motor_cmd()[slot].tau() == 0, "Nonarm slot received torque");

        full_cfg.arm_motor_indices[0] = full_cfg.arm_motor_indices[1];
        CheckRejected([&] { UnitreeController invalid(full_cfg, sink); }, "Duplicate arm slot accepted");
        full_cfg.arm_motor_indices[0] = kMotors;
        CheckRejected([&] { UnitreeController invalid(full_cfg, sink); }, "Out-of-range arm slot accepted");

        auto generic_cfg = Config(false, false);
        generic_cfg.arm_effort_enabled = true;
        generic_cfg.num_dofs = 6;
        generic_cfg.stiffness.resize(generic_cfg.num_dofs);
        generic_cfg.damping.resize(generic_cfg.num_dofs);
        generic_cfg.arm_motor_indices = {5, 1};
        generic_cfg.arm_total_torque_limits = {2, 2};
        generic_cfg.arm_effort_limits = {0.5, 0.5};
        UnitreeController generic(generic_cfg, sink);
        Feed(generic, 1);
        generic.arm_position_control();
        receipt = generic.step(std::vector<double>(6, 0), {0.25, -0.125}, generic.get_safety_status().reset_generation);
        const auto& motors = packets.back().motor_cmd();
        Check(receipt.arm_efforts == std::vector<float>{0.25f, -0.125f} &&
                  motors[5].tau() == 0.25f && motors[1].tau() == -0.125f && motors[0].tau() == 0,
              "Configured arm commands failed on a different motor layout");

        auto oversized_cfg = Config(false, false);
        oversized_cfg.num_dofs = LowCmd_{}.motor_cmd().size() + 1;
        oversized_cfg.stiffness.resize(oversized_cfg.num_dofs, 10);
        oversized_cfg.damping.resize(oversized_cfg.num_dofs, 1);
        CheckRejected([&] { UnitreeController invalid(oversized_cfg, sink); }, "Packet capacity was exceeded");
    }
};

int main() {
    try {
        UnitreeObserverTest::Run();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
