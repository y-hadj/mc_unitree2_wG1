
#include "Go2Control.h"

#include <mc_rtc/logging.h>

using namespace mc_unitree;

/**
 * @brief Interface constructor and destructor
 * DDS connection with robot using specified parameters.
 * Control level is set to LOW-level.
 *
 * @param config_param Configuration file parameters
 */
Go2Control::Go2Control(
    MCControlUnitree2<Go2Control, Go2SensorInfo, Go2CommandData,
                      Go2ConfigParameter>* mc_controller,
    mc_rbdyn::Robot* robot, const Go2ConfigParameter& config_param)
    : mc_controller_(mc_controller),
      robot_(robot),
      control_dt_(mc_controller->controller().timestep()),
      config_(config_param) {
  std::cout << "number of joints = " << robot_->refJointOrder().size()
            << std::endl;
  std::cout << "control dt = " << control_dt_ << std::endl;

  // q_init, q_lim_lower, q_lim_upper will be overwritten by definition in mc_h1
  // and urdf
  q_init_ = config_param.q_init_;
  q_lim_lower_ = config_param.q_lim_lower_;
  q_lim_upper_ = config_param.q_lim_upper_;
  kp_ = config_param.kp_;
  kd_ = config_param.kd_;
  kp_wait_ = config_param.kp_stand_;
  kd_wait_ = config_param.kd_stand_;
  tau_ff_ = config_param.tau_ff_;

  stateIn_.qIn_.resize(robot_->refJointOrder().size(), 0.0);
  stateIn_.dqIn_.resize(robot_->refJointOrder().size(), 0.0);
  stateIn_.tauIn_.resize(robot_->refJointOrder().size(), 0.0);
  stateIn_.rpyIn_.setZero();
  stateIn_.quatIn_.setIdentity();
  stateIn_.accIn_.setZero();
  stateIn_.rateIn_.setZero();
  stateIn_.footForceIn_.resize(robot_->forceSensors().size(), 0.0);

  cmdOut_.qOut_.resize(robot->refJointOrder().size(), 0.0);
  cmdOut_.dqOut_.resize(robot->refJointOrder().size(), 0.0);
  cmdOut_.tauOut_.resize(robot->refJointOrder().size(), 0.0);
  cmdOut_.kpOut_.resize(robot->refJointOrder().size());
  cmdOut_.kdOut_.resize(robot->refJointOrder().size());
  for (size_t i = 0; i < robot_->refJointOrder().size(); i++) {
    cmdOut_.kpOut_[i] = kp_[i];
    cmdOut_.kdOut_[i] = kd_[i];
  }

  for (size_t i = 0; i < robot->refJointOrder().size(); i++) {
    const std::string& jname = robot_->refJointOrder()[i];
    auto mcJointId = robot->jointIndexByName(jname);
    if (robot->mbc().q[mcJointId].empty()) continue;

    q_lim_lower_(i) = robot->ql().at(i)[0];
    q_lim_upper_(i) = robot->qu().at(i)[0];

    /* Overrite initial q_init_ if stance is set */
    if (robot->stance().count(jname)) {
      q_init_(i) = robot->stance().at(jname)[0];
      auto motorId = jointIdsToMotorIds[i];
      std::cout << jname << ": init=" << q_init_(i) << ", idx=" << i
                << ", mcJointId=" << mcJointId << ", motorId=" << motorId
                << std::endl;
    }
  }

  if (!config_param.network_.empty()) {
    /* Initialize */
    unitree::robot::ChannelFactory::Instance()->Init(0, config_param.network_);
    std::cout << "Initialize channel factory." << std::endl;

    lowcmd_publisher_.reset(
        new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(
            TOPIC_LOWCMD));
    lowcmd_publisher_->InitChannel();
    command_writer_ptr_ = unitree::common::CreateRecurrentThreadEx(
        "command_writer", UT_CPU_ID_NONE, 2000, &Go2Control::LowCommandWriter,
        this);

    lowstate_subscriber_.reset(
        new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(
            TOPIC_LOWSTATE));
    lowstate_subscriber_->InitChannel(
        std::bind(&Go2Control::LowStateHandler, this, std::placeholders::_1),
        1);

#if defined(__ENABLE_RT_PREEMPT__)
    pthread_create(
        &control_thread_, NULL,
        [](void* arg) -> void* {
          auto* ctrl = static_cast<Go2Control::Control*>(arg);
          ctrl->Control();
          return nullptr;
        },
        NULL);
#else
    int control_period_us = control_dt_ * 1e6;
    control_thread_ptr_ = unitree::common::CreateRecurrentThreadEx(
        "control", UT_CPU_ID_NONE, control_period_us, &Go2Control::Control,
        this);
#endif

    int report_period_us = report_dt_ * 1e6;
    report_sensor_ptr_ = unitree::common::CreateRecurrentThreadEx(
        "report_sensor", UT_CPU_ID_NONE, report_period_us,
        &Go2Control::UpdateTables, this, false);

    // Initialize tables for console display
    UpdateTables(true);
  }
}

Go2Control::~Go2Control() {
#if defined(__ENABLE_RT_PREEMPT__)
  pthread_join(control_thread_, NULL);
#endif
}

////
// WAITING BEFORE LAUNCHING CONTROLLER
////

// Wait for Enter key press
void waiting(Go2Control* controller) {
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(1000ms);
  std::cin.get();
  controller->endWaiting();
}

void Go2Control::endWaiting() {
  if (status_ == STATUS_WAITING_AIR) {
    status_ = STATUS_WAITING_GRD;
    std::thread wait_thread(waiting, this);
    wait_thread.detach();
  } else if (status_ == STATUS_WAITING_GRD) {
    time_run_ = -control_dt_;
    status_ = STATUS_GAIN_TRANSITION;
  }
}

void Go2Control::LowCommandWriter() {
  unitree_go::msg::dds_::LowCmd_ dds_low_command{};
  dds_low_command.head()[0] = 0xFE;
  dds_low_command.head()[1] = 0xEF;
  dds_low_command.level_flag() = 0xFF;
  dds_low_command.gpio() = 0;

  const std::shared_ptr<const MotorCommand> mc_tmp_ptr =
      motor_command_buffer_.GetData();
  if (mc_tmp_ptr) {
    for (int i = 0; i < kNumMotors; ++i) {
      dds_low_command.motor_cmd().at(i).mode() = (0x01);
      dds_low_command.motor_cmd().at(i).tau() = mc_tmp_ptr->tau_ff.at(i);
      dds_low_command.motor_cmd().at(i).q() = mc_tmp_ptr->q_ref.at(i);
      dds_low_command.motor_cmd().at(i).dq() = mc_tmp_ptr->dq_ref.at(i);
      dds_low_command.motor_cmd().at(i).kp() = mc_tmp_ptr->kp.at(i);
      dds_low_command.motor_cmd().at(i).kd() = mc_tmp_ptr->kd.at(i);
    }
    for (int i = kNumMotors; i < 20; ++i) {
      dds_low_command.motor_cmd().at(i).mode() = (0x01);
      dds_low_command.motor_cmd().at(i).tau() = (0);
      dds_low_command.motor_cmd().at(i).q() = (PosStopF);
      dds_low_command.motor_cmd().at(i).dq() = (VelStopF);
      dds_low_command.motor_cmd().at(i).kp() = (0);
      dds_low_command.motor_cmd().at(i).kd() = (0);
    }

    dds_low_command.crc() = Crc32Core((uint32_t*)&dds_low_command,
                                      (sizeof(dds_low_command) >> 2) - 1);
    lowcmd_publisher_->Write(dds_low_command);
  }
}

void Go2Control::LowStateHandler(const void* message) {
  unitree_go::msg::dds_::LowState_ low_state =
      *(unitree_go::msg::dds_::LowState_*)message;

  RecordMotorState(low_state);
  RecordBaseState(low_state);
}

void Go2Control::RecordMotorState(const unitree_go::msg::dds_::LowState_& msg) {
  MotorState ms_tmp;
  for (int i = 0; i < kNumMotors; ++i) {
    ms_tmp.q.at(i) = msg.motor_state()[i].q();
    ms_tmp.dq.at(i) = msg.motor_state()[i].dq();
    ms_tmp.tau.at(i) = msg.motor_state()[i].tau_est();
  }

  motor_state_buffer_.SetData(ms_tmp);
}

void Go2Control::RecordBaseState(const unitree_go::msg::dds_::LowState_& msg) {
  BaseState bs_tmp;
  bs_tmp.omega = msg.imu_state().gyroscope();
  bs_tmp.quat = msg.imu_state().quaternion();
  bs_tmp.rpy = msg.imu_state().rpy();
  bs_tmp.acc = msg.imu_state().accelerometer();

  base_state_buffer_.SetData(bs_tmp);
}

void Go2Control::ReportSensors() {
  const std::shared_ptr<const BaseState> bs_tmp_ptr =
      base_state_buffer_.GetData();
  const std::shared_ptr<const MotorState> ms_tmp_ptr =
      motor_state_buffer_.GetData();
  if (bs_tmp_ptr) {
    // Roll Pitch Yaw orientation
    std::cout << std::setprecision(4) << "rpy: [" << bs_tmp_ptr->rpy.at(0)
              << ", " << bs_tmp_ptr->rpy.at(1) << ", " << bs_tmp_ptr->rpy.at(2)
              << "]" << std::endl;
    // Gyroscope
    std::cout << std::setprecision(4) << "gyro: [" << bs_tmp_ptr->omega.at(0)
              << ", " << bs_tmp_ptr->omega.at(1) << ", "
              << bs_tmp_ptr->omega.at(2) << "]" << std::endl;
    // Accelerometer
    std::cout << std::setprecision(4) << "acc: [" << bs_tmp_ptr->acc.at(0)
              << ", " << bs_tmp_ptr->acc.at(1) << ", " << bs_tmp_ptr->acc.at(2)
              << "]" << std::endl;
  }
  if (ms_tmp_ptr) {
    // Joint positions
    std::cout << "mot_pos: [";
    for (size_t i = 0; i < robot_->refJointOrder().size(); ++i) {
      std::cout << std::setprecision(4)
                << ms_tmp_ptr->q.at(jointIdsToMotorIds[i]) << ", ";
    }
    std::cout << "]" << std::endl;

    // Joint velocities
    std::cout << "mot_vel: [";
    for (size_t i = 0; i < robot_->refJointOrder().size(); ++i) {
      std::cout << std::setprecision(4)
                << ms_tmp_ptr->dq.at(jointIdsToMotorIds[i]) << ", ";
    }
    std::cout << "]" << std::endl;
  }
}

void Go2Control::UpdateTables(bool init) {
  // Clear the console
  std::cout << u8"\033[2J";

  if (init) {
    // Set tables border style
    table_IMU_.set_border_style(FT_NICE_STYLE);
    table_legs_.set_border_style(FT_NICE_STYLE);
    table_legs_cmd_.set_border_style(FT_NICE_STYLE);
    table_misc_.set_border_style(FT_NICE_STYLE);

    // Initialize headers
    table_IMU_.set_cur_cell(0, 0);
    table_legs_.set_cur_cell(0, 0);
    table_legs_cmd_.set_cur_cell(0, 0);
    table_misc_.set_cur_cell(0, 0);
    table_IMU_ << fort::header << ""
               << "X"
               << "Y"
               << "Z" << fort::endr;
    table_legs_ << fort::header << ""
                << "L Yaw"
                << "L Roll"
                << "L Pitch"
                << "L Knee"
                << "L Ank";
    table_legs_ << "R Yaw"
                << "R Roll"
                << "R Pitch"
                << "R Knee"
                << "R Ank" << fort::endr;
    table_legs_cmd_ << fort::header << ""
                    << "L Yaw"
                    << "L Roll"
                    << "L Pitch"
                    << "L Knee"
                    << "L Ank";
    table_legs_cmd_ << "R Yaw"
                    << "R Roll"
                    << "R Pitch"
                    << "R Knee"
                    << "R Ank" << fort::endr;
    table_misc_ << fort::header << ""
                << "VX"
                << "VY"
                << "WZ" << fort::endr;
  }

  // Fill tables with data
  const std::shared_ptr<const BaseState> bs_tmp_ptr =
      base_state_buffer_.GetData();
  const std::shared_ptr<const MotorState> ms_tmp_ptr =
      motor_state_buffer_.GetData();

  // Set current cell to start of second row
  table_IMU_.set_cur_cell(1, 0);
  table_legs_.set_cur_cell(1, 0);
  table_legs_cmd_.set_cur_cell(1, 0);
  table_misc_.set_cur_cell(1, 0);

  // Fill IMU data
  if (bs_tmp_ptr) {
    table_IMU_ << "RPY";
    for (int i = 0; i < 3; ++i) {
      table_IMU_ << std::fixed << std::setprecision(4) << bs_tmp_ptr->rpy.at(i);
    }
    table_IMU_ << fort::endr << fort::separator << "Gyro";
    for (int i = 0; i < 3; ++i) {
      table_IMU_ << std::fixed << std::setprecision(4)
                 << bs_tmp_ptr->omega.at(i);
    }
    table_IMU_ << fort::endr << fort::separator << "Acc";
    for (int i = 0; i < 3; ++i) {
      table_IMU_ << std::fixed << std::setprecision(4) << bs_tmp_ptr->acc.at(i);
    }
  }

  // Fill joint data
  if (ms_tmp_ptr) {
    table_legs_ << "Pos";
    for (int i = 0; i < 12; ++i) {
      table_legs_ << std::fixed << std::setprecision(4)
                  << ms_tmp_ptr->q.at(jointIdsToMotorIds[i]);
    }
    table_legs_ << fort::endr << fort::separator << "Vel";
    for (int i = 0; i < 12; ++i) {
      table_legs_ << std::fixed << std::setprecision(4)
                  << ms_tmp_ptr->dq.at(jointIdsToMotorIds[i]);
    }
    table_legs_ << fort::endr << fort::separator << "Torques";
    for (int i = 0; i < 12; ++i) {
      table_legs_ << std::fixed << std::setprecision(4)
                  << ms_tmp_ptr->tau.at(jointIdsToMotorIds[i]);  // tau_des_[i];
    }
    table_legs_ << fort::endr;
  }

  const std::shared_ptr<const MotorCommand> mc_tmp_ptr =
      motor_command_buffer_.GetData();
  if (mc_tmp_ptr) {
    table_legs_cmd_ << "PosCmd";
    for (int i = 0; i < 12; ++i) {
      table_legs_cmd_ << std::fixed << std::setprecision(4)
                      << mc_tmp_ptr->q_ref.at(jointIdsToMotorIds[i]);
    }
    table_legs_cmd_ << fort::endr;
  }

  if (init) {
    // Set text style
    table_IMU_.row(0).set_cell_content_text_style(fort::text_style::bold);
    table_IMU_.column(0).set_cell_content_text_style(fort::text_style::bold);
    table_legs_.column(0).set_cell_content_text_style(fort::text_style::bold);
    table_legs_cmd_.column(0).set_cell_content_text_style(
        fort::text_style::bold);
    table_misc_.row(0).set_cell_content_text_style(fort::text_style::bold);
    table_misc_.column(0).set_cell_content_text_style(fort::text_style::bold);

    // Set alignment
    table_IMU_.column(0).set_cell_text_align(fort::text_align::center);
    for (int i = 1; i < 4; ++i) {
      table_IMU_.column(i).set_cell_text_align(fort::text_align::right);
      table_IMU_.column(i).set_cell_min_width(11);
    }
    table_IMU_[0][1].set_cell_text_align(fort::text_align::center);
    table_IMU_[0][2].set_cell_text_align(fort::text_align::center);
    table_IMU_[0][3].set_cell_text_align(fort::text_align::center);

    table_legs_.column(0).set_cell_text_align(fort::text_align::center);
    for (int i = 1; i < 13; ++i) {
      table_legs_.column(i).set_cell_text_align(fort::text_align::right);
      table_legs_.column(i).set_cell_min_width(9);
    }
    table_legs_cmd_.column(0).set_cell_text_align(fort::text_align::center);
    for (int i = 1; i < 13; ++i) {
      table_legs_cmd_.column(i).set_cell_text_align(fort::text_align::right);
      table_legs_cmd_.column(i).set_cell_min_width(9);
    }

    table_misc_.column(0).set_cell_text_align(fort::text_align::center);
    for (int i = 1; i < 4; ++i) {
      table_misc_.column(i).set_cell_text_align(fort::text_align::right);
      table_misc_.column(i).set_cell_min_width(9);
    }
    table_misc_[0][1].set_cell_text_align(fort::text_align::center);
    table_misc_[0][2].set_cell_text_align(fort::text_align::center);
    table_misc_[0][3].set_cell_text_align(fort::text_align::center);
  }

  switch (status_) {
    case STATUS_INIT:
      std::cout << "    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓" << std::endl;
      std::cout << "    ┃      Initialization      ┃" << std::endl;
      std::cout << "    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛" << std::endl << std::endl;
      break;
    case STATUS_WAITING_AIR:
      std::cout << "    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓" << std::endl;
      std::cout << "    ┃    Waiting in the air    ┃" << std::endl;
      std::cout << "    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛" << std::endl << std::endl;
      break;
    case STATUS_WAITING_GRD:
      std::cout << "    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓" << std::endl;
      std::cout << "    ┃   Waiting on the ground  ┃" << std::endl;
      std::cout << "    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛" << std::endl << std::endl;
      break;
    case STATUS_GAIN_TRANSITION:
      std::cout << "    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓" << std::endl;
      std::cout << "    ┃   PD Gains Transition    ┃" << std::endl;
      std::cout << "    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛" << std::endl << std::endl;
      break;
    case STATUS_RUN:
      std::cout << "    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓" << std::endl;
      std::cout << "    ┃    Running Controller    ┃" << std::endl;
      std::cout << "    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛" << std::endl << std::endl;
      break;
    case STATUS_DAMPING:
      std::cout << "    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓" << std::endl;
      std::cout << "    ┃    Emergency Damping!    ┃" << std::endl;
      std::cout << "    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛" << std::endl << std::endl;
      break;
  }
  std::cout << "    ┏━━━━━━━━━━━━━━━━━━━┓" << std::endl;
  std::cout << "    ┃    Sensor Data    ┃" << std::endl;
  std::cout << "    ┗━━━━━━━━━━━━━━━━━━━┛" << std::endl << std::endl;
  std::cout << table_IMU_.to_string() << std::endl;
  std::cout << table_legs_cmd_.to_string() << std::endl;
  std::cout << table_legs_.to_string() << std::endl;
  std::cout << table_misc_.to_string() << std::endl;
  std::cout << "Time: " << time_ << std::endl;
}

void Go2Control::Control() {
#if defined(__ENABLE_RT_PREEMPT__)
  if (set_sched_prio(RT_PRIO_MAX - 2, TASK_PERIOD) == -1) {
    perror("set_sched_prio failed on MCControlUnitree2.\n");
    return;
  } /* in art_process */
#endif

  MotorCommand motor_command_tmp;
  const std::shared_ptr<const MotorState> ms_tmp_ptr =
      motor_state_buffer_.GetData();
  const std::shared_ptr<const BaseState> bs_tmp_ptr =
      base_state_buffer_.GetData();

  if (!ms_tmp_ptr || !bs_tmp_ptr) return;

  time_ += control_dt_;

  Vector12 q_pos, q_vel;
  for (size_t i = 0; i < robot_->refJointOrder().size(); i++) {
    auto motorId = jointIdsToMotorIds[i];
    stateIn_.qIn_[i] = ms_tmp_ptr->q.at(motorId);
    stateIn_.dqIn_[i] = ms_tmp_ptr->dq.at(motorId);
    stateIn_.tauIn_[i] = ms_tmp_ptr->tau.at(motorId);

    q_pos[i] = ms_tmp_ptr->q.at(motorId);
    q_vel[i] = ms_tmp_ptr->dq.at(motorId);
  }
  q_pos[robot_->refJointOrder().size()] = 0.0;
  q_vel[robot_->refJointOrder().size()] = 0.0;

  for (int i = 0; i < 3; i++) {
    stateIn_.rpyIn_[i] = bs_tmp_ptr->rpy.at(i);
    stateIn_.accIn_[i] = bs_tmp_ptr->acc.at(i);
    stateIn_.rateIn_[i] = bs_tmp_ptr->omega.at(i);
  }
  for (int i = 0; i < 4; i++) {
    stateIn_.quatIn_.coeffs()[i] = bs_tmp_ptr->quat.at(i);
  }
#if 0
  for(int i = 0; i < stateIn_.footForceIn_.size(); i++)
  {
    stateIn_.footForceIn_[i] = (double)bs_tmp_ptr->foot_force()[i];
  }
#endif

  // Check if joints are too close from position limits
  const bool lim_lower = ((q_pos - 0.85 * q_lim_lower_).array() < 0.0).any();
  const bool lim_upper = ((q_pos - 0.85 * q_lim_upper_).array() > 0.0).any();
  if (lim_lower || lim_upper) {
    if (lim_lower) {
      std::cout << "Joint lower limit breached!!!" << std::endl;
      std::cout << "POS: " << std::fixed << std::setprecision(4)
                << q_pos.transpose() << std::endl;
      std::cout << "LIM: " << std::fixed << std::setprecision(4)
                << (0.85 * q_lim_lower_).transpose() << std::endl;
    }
    if (lim_upper) {
      std::cout << "Joint upper limit breached!!!" << std::endl;
      std::cout << "POS: " << std::fixed << std::setprecision(4)
                << q_pos.transpose() << std::endl;
      std::cout << "LIM: " << std::fixed << std::setprecision(4)
                << (0.85 * q_lim_upper_).transpose() << std::endl;
    }
    status_ = STATUS_DAMPING;
  }

  // Check if joint velocities are too high
  const bool lim_velocity = ((q_vel.array().abs() - 8) > 0.0).any();
  if (lim_velocity) {
    std::cout << "Velocity threshold breached!!!" << std::endl;
    std::cout << "VEL: " << std::fixed << std::setprecision(4)
              << q_vel.transpose() << std::endl;
    status_ = STATUS_DAMPING;
  }

  // Switch to waiting after initialization
  if ((status_ == STATUS_INIT) && (time_ > init_duration_)) {
    status_ = STATUS_WAITING_AIR;
    std::thread wait_thread(waiting, this);
    wait_thread.detach();
  }

  switch (status_) {
    case STATUS_RUN: {
      time_run_ += control_dt_;

      mc_controller_->run(stateIn_, cmdOut_);

      // Send commands to the robot
      for (size_t i = 0; i < robot_->refJointOrder().size(); ++i) {
        auto motorId = jointIdsToMotorIds[i];
        motor_command_tmp.kp.at(motorId) = cmdOut_.kpOut_[i];
        motor_command_tmp.kd.at(motorId) = cmdOut_.kdOut_[i];
        motor_command_tmp.q_ref.at(motorId) = cmdOut_.qOut_[i];
        motor_command_tmp.dq_ref.at(motorId) = cmdOut_.dqOut_[i];
        motor_command_tmp.tau_ff.at(motorId) = cmdOut_.tauOut_[i];
      }
      break;
    }

    case STATUS_WAITING_AIR: {
      // Wait at default configuration
      for (size_t i = 0; i < robot_->refJointOrder().size(); ++i) {
        auto motorId = jointIdsToMotorIds[i];
        motor_command_tmp.kp.at(motorId) = kp_wait_(i);
        motor_command_tmp.kd.at(motorId) = kd_wait_(i);
        motor_command_tmp.q_ref.at(motorId) = q_init_(i);
        motor_command_tmp.dq_ref.at(motorId) = 0.f;
        motor_command_tmp.tau_ff.at(motorId) = 0.f;
      }
      break;
    }

    case STATUS_WAITING_GRD: {
      // Wait at default configuration
      for (size_t i = 0; i < robot_->refJointOrder().size(); ++i) {
        auto motorId = jointIdsToMotorIds[i];
        motor_command_tmp.kp.at(motorId) = kp_wait_(i);
        motor_command_tmp.kd.at(motorId) = kd_wait_(i);
        motor_command_tmp.q_ref.at(motorId) = q_init_(i);
        motor_command_tmp.dq_ref.at(motorId) = 0.f;
        motor_command_tmp.tau_ff.at(motorId) = tau_ff_(i) * 0.0;
      }
      break;
    }

    case STATUS_GAIN_TRANSITION: {
      // Interpolation from waiting gains to policy gains
      time_run_ += control_dt_;

      // Slowly switch PD gains to policy gains
      float alpha = time_run_ / interp_duration_;
      for (size_t i = 0; i < robot_->refJointOrder().size(); ++i) {
        auto motorId = jointIdsToMotorIds[i];
        motor_command_tmp.kp.at(motorId) =
            kp_wait_(i) * (1 - alpha) + kp_(i) * alpha;
        motor_command_tmp.kd.at(motorId) =
            kd_wait_(i) * (1 - alpha) + kd_(i) * alpha;
        motor_command_tmp.q_ref.at(motorId) = q_init_(i);
        motor_command_tmp.dq_ref.at(motorId) = 0.f;
        motor_command_tmp.tau_ff.at(motorId) = 0.f;
      }

      // If transition is over, switch to the policy
      if (time_run_ >= interp_duration_) {
        time_run_ = -control_dt_;
        status_ = STATUS_RUN;
      }
      break;
    }

    case STATUS_INIT: {
      // Slowly move to default configuration
      float ratio = std::clamp(time_, 0.f, init_duration_) / init_duration_;
      for (size_t i = 0; i < robot_->refJointOrder().size(); ++i) {
        auto motorId = jointIdsToMotorIds[i];
        motor_command_tmp.kp.at(motorId) = kp_wait_(i);
        motor_command_tmp.kd.at(motorId) = kd_wait_(i);
        motor_command_tmp.dq_ref.at(motorId) = 0.f;
        motor_command_tmp.tau_ff.at(motorId) = 0.f;

        float q_des = (q_init_(i) - ms_tmp_ptr->q.at(motorId)) * ratio +
                      ms_tmp_ptr->q.at(motorId);
        motor_command_tmp.q_ref.at(motorId) = q_des;
      }
      break;
    }

    default: {  // case STATUS_DAMPING:
      // Emergency damping, no Kp, only Kd with 0 ref vel
      for (size_t i = 0; i < robot_->refJointOrder().size(); i++) {
        auto motorId = jointIdsToMotorIds[i];
        motor_command_tmp.kp.at(motorId) = 0.f;
        motor_command_tmp.kd.at(motorId) = kd_(i);
        motor_command_tmp.q_ref.at(motorId) = ms_tmp_ptr->q.at(motorId);
        motor_command_tmp.dq_ref.at(motorId) = 0.f;
        motor_command_tmp.tau_ff.at(motorId) = 0.f;
      }
    }
  }
  // Write to command buffer
  motor_command_buffer_.SetData(motor_command_tmp);

  // Log sensors and commands
  for (size_t i = 0; i < robot_->refJointOrder().size(); ++i) {
    auto motorId = jointIdsToMotorIds[i];
    tau_des_[i] =
        motor_command_tmp.kp.at(motorId) *
            (motor_command_tmp.q_ref.at(motorId) - ms_tmp_ptr->q.at(motorId)) +
        motor_command_tmp.kd.at(motorId) *
            (motor_command_tmp.dq_ref.at(motorId) -
             ms_tmp_ptr->dq.at(motorId)) +
        motor_command_tmp.tau_ff.at(motorId);
  }
}

/**
 * @brief Set initial state values for simulation
 *
 * @param stance Value defined by RobotModule
 * @param state Current sensor values information
 */
void Go2Control::setInitialState(
    const std::map<std::string, std::vector<double>>& stance) {
  /* Start stance */
  for (size_t i = 0; i < robot_->refJointOrder().size(); i++) {
    const std::string& jname = robot_->refJointOrder()[i];
    auto mcJointId = robot_->jointIndexByName(jname);
    auto jointId = mcJointIdToJointId(mcJointId);

    if (stance.count(jname)) {
      stateIn_.qIn_[jointId] = stance.at(jname)[0];
    } else {
      stateIn_.qIn_[jointId] = 0.0;
    }
    stateIn_.dqIn_[jointId] = 0.0;
    stateIn_.tauIn_[jointId] = 0.0;
  }

  /* Set body(imu) sensor values */
  stateIn_.rpyIn_.setZero();
  stateIn_.quatIn_.setIdentity();
  stateIn_.accIn_.setZero();
  stateIn_.rateIn_.setZero();
  for (int i = 0; i < stateIn_.footForceIn_.size(); i++) {
    stateIn_.footForceIn_[i] = 0.0;
  }
};

/**
 * @brief Loop back the value of "data" to "state"
 *
 * @param data Command data for sending to Go2 robot
 * @param state Current sensor values information
 */
void Go2Control::loopbackState(const Go2CommandData& data) {
  /*  Set current sensor values */
  for (size_t i = 0; i < robot_->refJointOrder().size(); i++) {
    stateIn_.qIn_[i] = data.qOut_[i];
    stateIn_.dqIn_[i] = data.dqOut_[i];
    stateIn_.tauIn_[i] = data.tauOut_[i];

    if (stateIn_.qIn_[i] > robot_->qu()[i][0])
      stateIn_.qIn_[i] = robot_->qu()[i][0];
    else if (stateIn_.qIn_[i] < robot_->ql()[i][0])
      stateIn_.qIn_[i] = robot_->ql()[i][0];
  }
}
