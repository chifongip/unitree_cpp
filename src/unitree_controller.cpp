#include "unitree_controller.hpp"
#include <algorithm>
#include <cstddef>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>

using std::size_t;

namespace {
bool FitsMotorFloat(double value) {
    return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
}

void ValidateGains(const std::vector<double>& stiffness, const std::vector<double>& damping, size_t count) {
    if (stiffness.size() != count || damping.size() != count)
        throw std::invalid_argument("stiffness and damping must match num_dofs");
    for (size_t i = 0; i < count; ++i) {
        if (!FitsMotorFloat(stiffness[i]) || !FitsMotorFloat(damping[i]) || stiffness[i] < 0 || damping[i] < 0)
            throw std::invalid_argument("stiffness and damping must be non-negative motor floats");
    }
}
}  // namespace

uint32_t UnitreeController::Crc32Core(uint32_t* ptr, uint32_t len) {
    uint32_t xbit = 0;
    uint32_t data = 0;
    uint32_t CRC32 = 0xFFFFFFFF;
    const uint32_t dwPolynomial = 0x04c11db7;
    for (uint32_t i = 0; i < len; i++) {
        xbit = 1 << 31;
        data = ptr[i];
        for (uint32_t bits = 0; bits < 32; bits++) {
            if (CRC32 & 0x80000000) {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            } else
                CRC32 <<= 1;
            if (data & xbit)
                CRC32 ^= dwPolynomial;

            xbit >>= 1;
        }
    }
    return CRC32;
};

UnitreeController::UnitreeController(const UnitreeConfig& cfg)
    : UnitreeController(cfg, {}) {}

UnitreeController::UnitreeController(const UnitreeConfig& cfg, std::function<bool(const LowCmd_&)> sink)
    : cfg_(cfg),
      command_sink_(std::move(sink)),
      stiffness_(cfg.stiffness),
      damping_(cfg.damping),
      num_dofs_(cfg.num_dofs),
      mode_pr_(Mode::PR),
      mode_machine_(0) {
    const auto motor_capacity = std::min(LowCmd_{}.motor_cmd().size(), LowState_{}.motor_state().size());
    if (num_dofs_ == 0 || num_dofs_ > motor_capacity) {
        throw std::invalid_argument("num_dofs must be within the Unitree packet motor capacity");
    }
    ValidateGains(stiffness_, damping_, num_dofs_);
    if (cfg_.command_timeout < 0.0 || !std::isfinite(cfg_.command_timeout)) {
        throw std::invalid_argument("command_timeout must be finite and non-negative");
    }
    if (cfg_.shutdown_damping < 0.0 || !FitsMotorFloat(cfg_.shutdown_damping)) {
        throw std::invalid_argument("shutdown_damping must be a non-negative motor float");
    }
    if (ArmCommandsConfigured() || cfg_.arm_effort_enabled ||
        !cfg_.arm_effort_limits.empty() || !cfg_.arm_total_torque_limits.empty()) {
        const auto arm_count = cfg_.arm_motor_indices.size();
        if (cfg_.msg_type != "hg" || cfg_.control_mode != ControlMode::POSITION ||
            cfg_.command_timeout <= 0 || !std::isfinite(cfg_.state_timeout) || cfg_.state_timeout <= 0 ||
            arm_count == 0 || cfg_.arm_total_torque_limits.size() != arm_count ||
            cfg_.arm_effort_limits.size() != arm_count) {
            throw std::invalid_argument("Unitree arm commands require a motor mapping, matching limits, and both watchdogs");
        }
        std::vector<bool> assigned(num_dofs_, false);
        for (size_t i = 0; i < arm_count; ++i) {
            const auto slot = cfg_.arm_motor_indices[i];
            if (slot >= num_dofs_ || assigned[slot]) {
                throw std::invalid_argument("Unitree arm motor indices must be unique and in range");
            }
            assigned[slot] = true;
            const double total = cfg_.arm_total_torque_limits[i];
            const double effort = cfg_.arm_effort_limits[i];
            if (!std::isfinite(total) || !FitsMotorFloat(effort) || total < 0 || effort < 0 ||
                effort > total || (cfg_.arm_effort_enabled && effort == 0)) {
                throw std::invalid_argument("Invalid Unitree arm torque limits");
            }
        }
    }
    if (cfg.hand_type == "Dex-3") {
        num_dofs_hand_ = 7;
    } else if (cfg.hand_type == "NONE") {
        num_dofs_hand_ = 0;
    } else {
        throw std::runtime_error("Unsupported hand type: " + cfg.hand_type);
    }
    if (command_sink_) {
        return;
    }
    std::cout << cfg.hand_type << " hand with " << num_dofs_hand_ << " DOFs." << std::endl;

    ChannelFactory::Instance()->Init(0, cfg_.net_if);
    std::cout << "UnitreeController initialized with network interface: " << cfg_.net_if << std::endl;

    // try to shutdown motion control-related service
    msc_ = std::make_shared<unitree::robot::b2::MotionSwitcherClient>();
    msc_->SetTimeout(5.0f);
    msc_->Init();
    std::string form, name;
    while (msc_->CheckMode(form, name), !name.empty()) {
        if (msc_->ReleaseMode())
            std::cout << "Failed to switch to Release Mode\n";
        sleep(1);
    }
    std::cout << "Motion control service shutdown successfully." << std::endl;

    // create publisher
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(cfg_.lowcmd_topic));  // TODO: switch Cmd Type
    lowcmd_publisher_->InitChannel();
    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(cfg_.lowstate_topic));
    lowstate_subscriber_->InitChannel(std::bind(&UnitreeController::LowStateHandler, this, std::placeholders::_1), 1);

    if (cfg_.enable_torso_imu) {
        std::cout << "Torso IMU enabled, subscribing to topic: " << cfg_.torso_imu_topic << std::endl;
        torso_imu_subscriber_.reset(new ChannelSubscriber<unitree_hg::msg::dds_::IMUState_>(cfg_.torso_imu_topic));
        torso_imu_subscriber_->InitChannel(std::bind(&UnitreeController::TorsoImuStateHandler, this, std::placeholders::_1), 1);
    } else {
        std::cout << "Torso IMU disabled." << std::endl;
    }

    if (cfg_.enable_odometry) {
        std::cout << "Odometry enabled, subscribing to sport state topic: " << cfg_.sport_state_topic << std::endl;
        estimate_state_subscriber.reset(new ChannelSubscriber<SportModeState_>(cfg_.sport_state_topic));
        estimate_state_subscriber->InitChannel(std::bind(&UnitreeController::SportStateHandler, this, std::placeholders::_1), 1);
    } else {
        std::cout << "Odometry disabled." << std::endl;
    }

    // std::string sub_namespace = "rt/dex3/left/state";
    // unitree_hg::msg::dds_::HandState_ state;
    // handstate_subscriber.reset(new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::HandState_>(sub_namespace));

    // create threads
    command_writer_ptr_ = CreateRecurrentThreadEx("command_writer", UT_CPU_ID_NONE, uint(cfg.control_dt * 1e6), &UnitreeController::LowCommandWriter, this);

    handcmd_left_publisher_.reset(new ChannelPublisher<HandCmd_>("rt/dex3/left/cmd"));
    handcmd_left_publisher_->InitChannel();
    handcmd_right_publisher_.reset(new ChannelPublisher<HandCmd_>("rt/dex3/right/cmd"));
    handcmd_right_publisher_->InitChannel();
    handcmd_writer_ptr_ = CreateRecurrentThreadEx("handcmd_writer", UT_CPU_ID_NONE, uint(cfg.control_dt * 1e6 * 5), &UnitreeController::HandCommandWriter, this);

    init_done_ = true;
}

UnitreeController::~UnitreeController() {
}

bool UnitreeController::self_check() {
    if (!init_done_) {
        std::cerr << "UnitreeController not initialized properly." << std::endl;
        return false;
    }
    try {
        RobotState robot_state = get_robot_state();
        if (robot_state.tick == 0) {
            std::cerr << "Robot state tick is zero, no data received." << std::endl;
            return false;
        }
        if (cfg_.enable_odometry) {
            SportState sport_state = get_sport_state();
            if (sport_state.position.empty() || sport_state.velocity.empty()) {
                std::cerr << "Sport state data is empty." << std::endl;
                return false;
            }
        }
    } catch (const std::runtime_error& e) {
        std::cerr << "No data available: " << e.what() << std::endl;
        return false;
    }
    std::cout << "UnitreeController self-check passed." << std::endl;
    return true;
}

void UnitreeController::LowStateHandler(const void* message) {
    const auto received_ns = MonotonicNowNs();
    LowState_ low_state = *(const LowState_*)message;
    if (low_state.crc() != Crc32Core((uint32_t*)&low_state, (sizeof(LowState_) >> 2) - 1)) {
        std::cout << "[ERROR] CRC Error" << std::endl;
        return;
    }
    RobotState robot_state_tmp(num_dofs_);
    robot_state_tmp.tick = low_state.tick();

    MotorState& ms_tmp = robot_state_tmp.motor_state;
    for (int i = 0; i < num_dofs_; ++i) {
        ms_tmp.q.at(i) = low_state.motor_state()[i].q();
        ms_tmp.dq.at(i) = low_state.motor_state()[i].dq();
        ms_tmp.tau_est.at(i) = low_state.motor_state()[i].tau_est();
    }

    ImuState& imu_tmp = robot_state_tmp.imu_state;
    imu_tmp.quaternion = low_state.imu_state().quaternion();
    imu_tmp.gyroscope = low_state.imu_state().gyroscope();
    imu_tmp.accelerometer = low_state.imu_state().accelerometer();
    imu_tmp.rpy = low_state.imu_state().rpy();

    memcpy(&robot_state_tmp.wireless_remote, &low_state.wireless_remote()[0], 40);

    if (cfg_.enable_torso_imu) {
        const auto torso_imu = torso_imu_state_buffer_.GetData();
        if (torso_imu) {
            robot_state_tmp.torso_imu_state = *torso_imu;
        }
    }

    std::unique_lock<std::mutex> lock(command_mutex_, std::defer_lock);
    if (ArmCommandsConfigured()) {
        lock.lock();
        const auto previous = robot_state_buffer_.GetData();
        if (previous) {
            const uint32_t tick_delta = robot_state_tmp.tick - previous->tick;
            if (received_ns < last_state_time_ns_.load() || tick_delta == 0)
                return;
            if (tick_delta >= (uint32_t{1} << 31))
                LatchDampingLocked();
        }
    }
    robot_state_tmp.received_time_sec = received_ns * 1e-9;
    robot_state_buffer_.SetData(robot_state_tmp);
    last_state_time_ns_.store(received_ns);

    // update mode machine
    if (mode_machine_ != low_state.mode_machine()) {
        if (mode_machine_ == 0)
            std::cout << "Unitree mode_machine: " << unsigned(low_state.mode_machine()) << std::endl;
        mode_machine_ = low_state.mode_machine();
    }
}

void UnitreeController::TorsoImuStateHandler(const void* message) {
    using unitree_hg::msg::dds_::IMUState_;
    const IMUState_& dds_imu = *(const IMUState_*)message;

    ImuState imu_tmp;
    imu_tmp.rpy = dds_imu.rpy();
    imu_tmp.gyroscope = dds_imu.gyroscope();
    imu_tmp.quaternion = dds_imu.quaternion();
    imu_tmp.accelerometer = dds_imu.accelerometer();
    torso_imu_state_buffer_.SetData(imu_tmp);
}

void UnitreeController::SportStateHandler(const void* message) {
    SportModeState_ estimator_state = *(const SportModeState_*)message;

    SportState sport_state_tmp;
    sport_state_tmp.position = estimator_state.position();
    sport_state_tmp.velocity = estimator_state.velocity();
    sport_state_buffer_.SetData(sport_state_tmp);
}

void UnitreeController::LowCommandWriter() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    LowCommandWriterLocked();
}

void UnitreeController::LatchDampingLocked() {
    if (watchdog_latched_ && command_mode_.load() == CommandMode::DAMPING)
        return;
    watchdog_latched_ = true;
    ++reset_generation_;
    motor_command_buffer_.SetData(MakeDampingCommand(cfg_.shutdown_damping));
    command_mode_.store(CommandMode::DAMPING);
}

void UnitreeController::CheckArmCommandWatchdogsLocked(int64_t now) {
    if (!ArmCommandsConfigured() || command_mode_.load() != CommandMode::POSITION)
        return;
    const auto received = last_state_time_ns_.load();
    if (received <= 0 || now - received > cfg_.state_timeout * 1e9 ||
        now - last_command_time_ns_.load() > cfg_.command_timeout * 1e9) {
        LatchDampingLocked();
    }
}

void UnitreeController::RequirePositionControlLocked(int64_t now) {
    CheckArmCommandWatchdogsLocked(now);
    if (shutdown_started_ || watchdog_latched_ || command_mode_.load() != CommandMode::POSITION) {
        LowCommandWriterLocked();
        throw std::runtime_error("Unitree arm position control requires explicit rearm");
    }
}

void UnitreeController::LowCommandWriterLocked() {
    if (ArmCommandsConfigured()) {
        CheckArmCommandWatchdogsLocked(MonotonicNowNs());
    } else if (command_mode_.load() == CommandMode::POSITION && cfg_.command_timeout > 0.0 &&
               MonotonicNowNs() - last_command_time_ns_.load() > cfg_.command_timeout * 1e9) {
        motor_command_buffer_.SetData(MakeDampingCommand(cfg_.shutdown_damping));
        command_mode_.store(CommandMode::DAMPING);
        std::cerr << "Unitree position-command watchdog expired; damping enabled." << std::endl;
    }
    if (!WriteCommandLocked() && ArmCommandsConfigured()) {
        LatchDampingLocked();
        WriteCommandLocked();
    }
}

bool UnitreeController::WriteCommandLocked() {
    LowCmd_ dds_low_command;
    dds_low_command.mode_pr() = static_cast<uint8_t>(mode_pr_);
    dds_low_command.mode_machine() = mode_machine_;

    const auto mc = motor_command_buffer_.GetData();
    if (mc) {
        for (size_t i = 0; i < num_dofs_; i++) {
            dds_low_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
            dds_low_command.motor_cmd().at(i).tau() = mc->tau_ff.at(i);
            dds_low_command.motor_cmd().at(i).q() = mc->q_target.at(i);
            dds_low_command.motor_cmd().at(i).dq() = mc->dq_target.at(i);
            dds_low_command.motor_cmd().at(i).kp() = mc->kp.at(i);
            dds_low_command.motor_cmd().at(i).kd() = mc->kd.at(i);
        }

        dds_low_command.crc() = Crc32Core((uint32_t*)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
        try {
            return command_sink_ ? command_sink_(dds_low_command) : lowcmd_publisher_->Write(dds_low_command);
        } catch (const std::exception&) {
            if (!ArmCommandsConfigured())
                throw;
            return false;
        }
    }
    return true;
}

void UnitreeController::HandCommandWriter() {
    HandCmd_ dds_hand_command;

    dds_hand_command.motor_cmd().resize(num_dofs_hand_);

    const std::shared_ptr<const HandCommand> hc_l = hand_command_left_buffer_.GetData();
    if (hc_l) {
        // std::cout << "LowCommandWriter called with motor command data." << std::endl;
        for (size_t i = 0; i < num_dofs_hand_; i++) {
            dds_hand_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
            dds_hand_command.motor_cmd().at(i).tau() = hc_l->tau_ff.at(i);
            dds_hand_command.motor_cmd().at(i).q() = hc_l->q_target.at(i);
            dds_hand_command.motor_cmd().at(i).dq() = hc_l->dq_target.at(i);
            dds_hand_command.motor_cmd().at(i).kp() = hc_l->kp.at(i);
            dds_hand_command.motor_cmd().at(i).kd() = hc_l->kd.at(i);
        }

        handcmd_left_publisher_->Write(dds_hand_command);
    }
    const std::shared_ptr<const HandCommand> hc_r = hand_command_right_buffer_.GetData();
    if (hc_r) {
        // std::cout << "LowCommandWriter called with motor command data." << std::endl;
        for (size_t i = 0; i < num_dofs_hand_; i++) {
            dds_hand_command.motor_cmd().at(i).mode() = 1;  // 1:Enable, 0:Disable
            dds_hand_command.motor_cmd().at(i).tau() = hc_r->tau_ff.at(i);
            dds_hand_command.motor_cmd().at(i).q() = hc_r->q_target.at(i);
            dds_hand_command.motor_cmd().at(i).dq() = hc_r->dq_target.at(i);
            dds_hand_command.motor_cmd().at(i).kp() = hc_r->kp.at(i);
            dds_hand_command.motor_cmd().at(i).kd() = hc_r->kd.at(i);
        }

        handcmd_right_publisher_->Write(dds_hand_command);
    }
}

void UnitreeController::step(const std::vector<double>& actions) {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (ArmCommandsConfigured())
        RequirePositionControlLocked(MonotonicNowNs());
    if (actions.size() != num_dofs_ || !std::all_of(actions.begin(), actions.end(), FitsMotorFloat)) {
        if (ArmCommandsConfigured()) {
            LatchDampingLocked();
            WriteCommandLocked();
        }
        throw std::invalid_argument("actions must match num_dofs and contain only motor floats");
    }

    MotorCommand motor_command_tmp(num_dofs_);

    for (int i = 0; i < num_dofs_; ++i) {
        motor_command_tmp.kp.at(i) = stiffness_[i];
        motor_command_tmp.kd.at(i) = damping_[i];
        switch (cfg_.control_mode) {
            case ControlMode::POSITION:
                motor_command_tmp.q_target.at(i) = actions[i];
                break;
            case ControlMode::VELOCITY:
                motor_command_tmp.dq_target.at(i) = actions[i];
                break;
            case ControlMode::TORQUE:
                motor_command_tmp.tau_ff.at(i) = actions[i];
                break;
            default:
                throw std::runtime_error("Unknown control mode");
        }

    }
    motor_command_buffer_.SetData(motor_command_tmp);
    last_command_time_ns_.store(MonotonicNowNs());
    command_mode_.store(CommandMode::POSITION);
    LowCommandWriterLocked();
    if (ArmCommandsConfigured() && watchdog_latched_)
        throw std::runtime_error("Unitree position command failed; damping is latched");
}

UnitreeCommandReceipt UnitreeController::step(
    const std::vector<double>& positions, const std::vector<double>& arm_efforts, uint64_t expected_reset_generation) {
    if (!ArmCommandsConfigured())
        throw std::runtime_error("Unitree arm commands are not configured");
    std::lock_guard<std::mutex> lock(command_mutex_);
    RequirePositionControlLocked(MonotonicNowNs());
    UnitreeCommandReceipt receipt{};
    try {
        const auto arm_count = cfg_.arm_motor_indices.size();
        if (positions.size() != num_dofs_ || arm_efforts.size() != arm_count)
            throw std::invalid_argument("Unitree position or arm effort command size mismatch");
        const auto state = robot_state_buffer_.GetData();
        MotorCommand command(num_dofs_);
        for (size_t i = 0; i < num_dofs_; ++i) {
            if (!FitsMotorFloat(positions[i]))
                throw std::invalid_argument("Invalid Unitree position target");
            command.q_target[i] = static_cast<float>(positions[i]);
            command.kp[i] = static_cast<float>(stiffness_[i]);
            command.kd[i] = static_cast<float>(damping_[i]);
        }
        const bool accept_effort = cfg_.arm_effort_enabled && expected_reset_generation == reset_generation_;
        receipt.arm_efforts.assign(arm_count, 0.0f);
        for (size_t arm = 0; arm < arm_count; ++arm) {
            const double requested = arm_efforts[arm];
            if (!std::isfinite(requested))
                throw std::invalid_argument("Invalid Unitree arm effort");
            if (requested == 0 || !accept_effort)
                continue;
            const auto i = cfg_.arm_motor_indices[arm];
            const double q = state->motor_state.q[i];
            const double dq = state->motor_state.dq[i];
            const double pd = command.kp[i] * (command.q_target[i] - q) - command.kd[i] * dq;
            if (!std::isfinite(pd))
                throw std::invalid_argument("Invalid Unitree arm feedback or gains");
            const double total = cfg_.arm_total_torque_limits[arm];
            const double room = std::max(0.0, total - std::copysign(1.0, requested) * pd);
            const double admitted = std::copysign(
                std::min({std::abs(requested), cfg_.arm_effort_limits[arm], room}), requested);
            float effort = static_cast<float>(admitted);
            // The stored effort must not exceed the admitted magnitude.
            if (std::abs(effort) > std::abs(admitted))
                effort = std::nextafter(effort, 0.0f);
            command.tau_ff[i] = effort;
            receipt.arm_efforts[arm] = effort;
        }
        motor_command_buffer_.SetData(command);
        const auto accepted = MonotonicNowNs();
        last_command_time_ns_.store(accepted);
        receipt.command_time_sec = accepted * 1e-9;
        receipt.reset_generation = reset_generation_;
    } catch (const std::exception&) {
        LatchDampingLocked();
        WriteCommandLocked();
        throw;
    }
    LowCommandWriterLocked();
    if (watchdog_latched_)
        throw std::runtime_error("Unitree arm command failed; damping is latched");
    return receipt;
}

UnitreeSafetyStatus UnitreeController::get_safety_status() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    CheckArmCommandWatchdogsLocked(MonotonicNowNs());
    return {reset_generation_, watchdog_latched_};
}

void UnitreeController::clear_arm_effort() {
    if (!ArmCommandsConfigured())
        return;
    std::lock_guard<std::mutex> lock(command_mutex_);
    ClearArmEffortLocked();
}

void UnitreeController::ClearArmEffortLocked() {
    ++reset_generation_;
    const auto current = motor_command_buffer_.GetData();
    if (current && std::any_of(current->tau_ff.begin(), current->tau_ff.end(), [](float x) { return x != 0; })) {
        auto command = *current;
        std::fill(command.tau_ff.begin(), command.tau_ff.end(), 0.0f);
        motor_command_buffer_.SetData(command);
        LowCommandWriterLocked();
    }
}

void UnitreeController::step_hands(const std::vector<double>& l_hand_pose, const std::vector<double>& r_hand_pose) {
    if (l_hand_pose.size() != num_dofs_hand_ || r_hand_pose.size() != num_dofs_hand_) {
        throw std::runtime_error("l_hand_pose or r_hand_pose size mismatch");
    }

    HandCommand hand_command_left_tmp(num_dofs_hand_);

    for (int i = 0; i < num_dofs_hand_; ++i) {
        hand_command_left_tmp.q_target.at(i) = l_hand_pose[i];
        hand_command_left_tmp.dq_target.at(i) = 0.0;
        hand_command_left_tmp.kp.at(i) = 1.5f;
        hand_command_left_tmp.kd.at(i) = 0.1f;
        hand_command_left_tmp.tau_ff.at(i) = 0.0f;
    }
    hand_command_left_buffer_.SetData(hand_command_left_tmp);

    HandCommand hand_command_right_tmp(num_dofs_hand_);
    for (int i = 0; i < num_dofs_hand_; ++i) {
        hand_command_right_tmp.q_target.at(i) = r_hand_pose[i];
        hand_command_right_tmp.dq_target.at(i) = 0.0;
        hand_command_right_tmp.kp.at(i) = 1.5f;
        hand_command_right_tmp.kd.at(i) = 0.1f;
        hand_command_right_tmp.tau_ff.at(i) = 0.0f;
    }
    hand_command_right_buffer_.SetData(hand_command_right_tmp);
    HandCommandWriter(); // immediately send command
}

void UnitreeController::set_gains(const std::vector<double>& stiffness, const std::vector<double>& damping) {
    ValidateGains(stiffness, damping, num_dofs_);
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        stiffness_ = stiffness;
        damping_ = damping;
        if (ArmCommandsConfigured())
            ClearArmEffortLocked();
    }

    std::cout << "Gains set: stiffness = [";
    for (const auto& s : stiffness) {
        std::cout << s << " ";
    }
    std::cout << "], damping = [";
    for (const auto& d : damping) {
        std::cout << d << " ";
    }
    std::cout << "]" << std::endl;
}

void UnitreeController::set_passive() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    CheckArmCommandWatchdogsLocked(MonotonicNowNs());
    if (!watchdog_latched_) {
        motor_command_buffer_.SetData(MotorCommand(num_dofs_));
        command_mode_.store(CommandMode::PASSIVE);
        ++reset_generation_;
    }
    LowCommandWriterLocked();
}

MotorCommand UnitreeController::MakeDampingCommand(double damping) const {
    if (!FitsMotorFloat(damping) || damping < 0.0) {
        throw std::invalid_argument("damping must be a non-negative motor float");
    }
    MotorCommand damping_command(num_dofs_);
    std::fill(damping_command.kd.begin(), damping_command.kd.end(), static_cast<float>(damping));
    return damping_command;
}

void UnitreeController::set_damping(double damping) {
    const auto damping_command = MakeDampingCommand(damping);
    std::lock_guard<std::mutex> lock(command_mutex_);
    CheckArmCommandWatchdogsLocked(MonotonicNowNs());
    if (!watchdog_latched_) {
        motor_command_buffer_.SetData(damping_command);
        command_mode_.store(CommandMode::DAMPING);
        ++reset_generation_;
    }
    LowCommandWriterLocked();
}

void UnitreeController::arm_position_control() {
    MotorCommand hold_command(num_dofs_);
    {
        std::lock_guard<std::mutex> lock(command_mutex_);
        if (ArmCommandsConfigured() && (shutdown_started_ || !state_is_fresh(cfg_.state_timeout))) {
            LatchDampingLocked();
            WriteCommandLocked();
            throw std::runtime_error("Unitree arm rearm requires fresh feedback and a running controller");
        }
        const RobotState robot_state = get_robot_state();
        for (size_t i = 0; i < num_dofs_; ++i) {
            const bool controlled = stiffness_[i] > 0 || damping_[i] > 0;
            if (ArmCommandsConfigured() && controlled && !std::isfinite(robot_state.motor_state.q[i])) {
                LatchDampingLocked();
                WriteCommandLocked();
                throw std::runtime_error("Unitree arm rearm received nonfinite joint positions");
            }
            hold_command.q_target[i] =
                !ArmCommandsConfigured() || controlled ? robot_state.motor_state.q[i] : 0.0f;
            hold_command.kp[i] = stiffness_[i];
            hold_command.kd[i] = damping_[i];
        }
        motor_command_buffer_.SetData(hold_command);
        last_command_time_ns_.store(MonotonicNowNs());
        command_mode_.store(CommandMode::POSITION);
        watchdog_latched_ = false;
        ++reset_generation_;
        LowCommandWriterLocked();
        if (ArmCommandsConfigured() && watchdog_latched_)
            throw std::runtime_error("Unitree rearm failed; damping is latched");
    }
}

int64_t UnitreeController::MonotonicNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

bool UnitreeController::state_is_fresh(double timeout) const {
    if (!std::isfinite(timeout) || timeout < 0.0) {
        throw std::invalid_argument("state timeout must be finite and non-negative");
    }
    if (timeout == 0.0) {
        return true;
    }
    const int64_t last_state_ns = last_state_time_ns_.load();
    return last_state_ns > 0 && MonotonicNowNs() - last_state_ns <= static_cast<int64_t>(timeout * 1e9);
}

void UnitreeController::shutdown() {
    std::cout << "Shutting down UnitreeController..." << std::endl;
    if (ArmCommandsConfigured()) {
        std::lock_guard<std::mutex> lock(command_mutex_);
        shutdown_started_ = true;
        LatchDampingLocked();
        WriteCommandLocked();
        return;
    }
    set_damping(cfg_.shutdown_damping);
}

RobotState UnitreeController::get_robot_state() {
    const std::shared_ptr<const RobotState> robot_state = robot_state_buffer_.GetData();

    if (robot_state) {
        auto snapshot = *robot_state;
        snapshot.age_sec = std::max(0.0, MonotonicNowNs() * 1e-9 - snapshot.received_time_sec);
        return snapshot;
    } else {
        throw std::runtime_error("Low state data is not available");
    }
}

SportState UnitreeController::get_sport_state() {
    const std::shared_ptr<const SportState> sport_state = sport_state_buffer_.GetData();

    if (sport_state) {
        return *sport_state;
    } else {
        throw std::runtime_error("Sport state data is not available");
    }
}

#ifdef UNITREE_CPP_STANDALONE
int main(int argc, char const* argv[]) {
    // Example usage of UnitreeController
    UnitreeConfig config;
    config.net_if = "enp13s0";
    config.control_dt = 0.1;
    config.msg_type = "hg";
    config.control_mode = ControlMode::POSITION;
    config.hand_type = "Dex-3";
    config.lowcmd_topic = "rt/lowcmd";
    config.lowstate_topic = "rt/lowstate";
    config.enable_odometry = false;
    config.sport_state_topic = "rt/odommodestate";
    config.enable_torso_imu = true;
    config.torso_imu_topic = "rt/secondary_imu";
    config.stiffness = {1.0, 1.0, 1.0};  // Example stiffness values
    config.damping = {0.1, 0.1, 0.1};    // Example damping values
    config.num_dofs = 3;                 // Example number of DOFs

    UnitreeController controller(config);

    while (true)
        sleep(10);
    return 0;
}
#endif
