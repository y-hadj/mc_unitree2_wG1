#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include "G1Revo2Control.h"
#include <RBDyn/FD.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <mc_rtc/path.h>
#include <mc_rtc/gui/Robot.h>
#include <filesystem>
#include <iostream>

using namespace mc_unitree;

/**
 * @brief Interface constructor and destructor
 * DDS connection with robot using specified parameters.
 * Control level is set to LOW-level.
 *
 * @param config_param Configuration file parameters
 */
G1Revo2Control::G1Revo2Control(MCControlUnitree2<G1Revo2Control, G1Revo2SensorInfo, G1Revo2CommandData, G1Revo2ConfigParameter> * mc_controller, mc_rbdyn::Robot * robot, const G1Revo2ConfigParameter & config_param)
  : mc_controller_(mc_controller), robot_(robot), control_dt_(mc_controller->controller().timestep()), config_(config_param)
{
  mc_rtc::log::info("[mc_unitree] G1Revo2 Number of joints: {}", robot_->refJointOrder().size());
  mc_rtc::log::info("[mc_unitree] G1Revo2 Control dt: {}", control_dt_);

  // mc_control::MCGlobalController::GlobalConfiguration gconfig("", nullptr);
  auto & controller = mc_controller_->controller();
  auto gconfig = controller.configuration();
  if(!gconfig.config.has("Unitree"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_unitree] Missing Unitree configuration. Dumping the loaded config file: {}", gconfig.config.dump());
  }
  auto unitree_config = gconfig.config("Unitree");
  if(!unitree_config.has("g1_29dof_revo2"))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[mc_unitree] Missing Unitree G1 configuration");
  }
  auto g1_config = unitree_config("g1_29dof_revo2");
  
  // q_init, q_lim_lower, q_lim_upper will be overwritten by definition in mc_g1 and urdf
  if(g1_config.has("q_init"))
  {
    Eigen::VectorXd q_init = g1_config("q_init");
    if(q_init.size() != 41) q_init_ = config_param.q_init_;
    else q_init_ = q_init.cast<float>();
  }
  else q_init_ = config_param.q_init_;

  if(g1_config.has("q_lim_lower"))
  {
    Eigen::VectorXd q_lim_lower = g1_config("q_lim_lower");
    if(q_lim_lower.size() != 41) q_lim_lower_ = config_param.q_lim_lower_;
    else q_lim_lower_ = q_lim_lower.cast<float>();
  }
  else q_lim_lower_ = config_param.q_lim_lower_;

  if(g1_config.has("q_lim_upper"))
  {
    Eigen::VectorXd q_lim_upper = g1_config("q_lim_upper");
    if(q_lim_upper.size() != 41) q_lim_upper_ = config_param.q_lim_upper_;
    else q_lim_upper_ = q_lim_upper.cast<float>();
  }
  else q_lim_upper_ = config_param.q_lim_upper_;

  if(g1_config.has("qdot_lim"))
  {
    Eigen::VectorXd qdot_lim_upper = g1_config("qdot_lim");
    Eigen::VectorXd qdot_lim_lower = -1.0 * qdot_lim_upper;
    if(qdot_lim_upper.size() != 41){
      q_dot_lim_upper_ = config_param.qdot_lim_;
      q_dot_lim_lower_ = -1.0 * config_param.qdot_lim_;
    }
    else{
      q_dot_lim_upper_ = qdot_lim_upper.cast<float>();
      q_dot_lim_lower_ = qdot_lim_lower.cast<float>();
    }
  }
  else {
    q_dot_lim_lower_ = -1.0 * config_param.qdot_lim_;
    q_dot_lim_upper_ = config_param.qdot_lim_;
  }

  if(g1_config.has("kp"))
  {
    Eigen::VectorXd kp = g1_config("kp");
    if(kp.size() != 41) kp_ = config_param.kp_;
    else kp_ = kp.cast<float>();
  }
  else kp_ = config_param.kp_;

  if(g1_config.has("kd"))
  {
    Eigen::VectorXd kd = g1_config("kd");
    if(kd.size() != 41) kd_ = config_param.kd_;
    else kd_ = kd.cast<float>();
  }
  else kd_ = config_param.kd_;

  if(g1_config.has("kp_wait"))
  {
    Eigen::VectorXd kp_wait = g1_config("kp_wait");
    if(kp_wait.size() != 41) kp_wait_ = config_param.kp_stand_;
    else kp_wait_ = kp_wait.cast<float>();
  }
  else kp_wait_ = config_param.kp_stand_;

  if(g1_config.has("kd_wait"))
  {
    Eigen::VectorXd kd_wait = g1_config("kd_wait");
    if(kd_wait.size() != 41) kd_wait_ = config_param.kd_stand_;
    else kd_wait_ = kd_wait.cast<float>();
  }
  else kd_wait_ = config_param.kd_stand_;

  if(g1_config.has("tau_ff"))
  {
    Eigen::VectorXd tau_ff = g1_config("tau_ff");
    if(tau_ff.size() != 41) tau_ff_ = config_param.tau_ff_;
    else tau_ff_ = tau_ff.cast<float>();
  }
  else tau_ff_ = config_param.tau_ff_;

  stateIn_.qIn_.resize(robot->refJointOrder().size(), 0.0);
  stateIn_.dqIn_.resize(robot->refJointOrder().size(), 0.0);
  stateIn_.tauIn_.resize(robot->refJointOrder().size(), 0.0);
  stateIn_.rpyIn_.setZero();
  stateIn_.quatIn_.setIdentity();
  stateIn_.accIn_.setZero();
  stateIn_.rateIn_.setZero();
  
  cmdOut_.qOut_.resize(robot->refJointOrder().size(), 0.0);
  cmdOut_.dqOut_.resize(robot->refJointOrder().size(), 0.0);
  cmdOut_.tauOut_.resize(robot->refJointOrder().size(), 0.0);
  cmdOut_.kpOut_.resize(robot->refJointOrder().size());
  cmdOut_.kdOut_.resize(robot->refJointOrder().size());
  
  // map refJointOrder <-> motor id by joint name
  rjoToMotorId_.assign(robot->refJointOrder().size(), -1);
  motorIdToRjo_.fill(-1);
  for (size_t i = 0 ; i < robot->refJointOrder().size() ; i++)
  {
    const std::string & jname = robot->refJointOrder()[i];
    for (size_t m = 0 ; m < static_cast<size_t>(kNumMotors) ; ++m)
    {
      if (jname == motorJointNames[m])
      {
        rjoToMotorId_[i] = static_cast<int>(m);
        motorIdToRjo_[m] = static_cast<int>(i);
        break;
      }
    }
  }
  for (size_t m = 0 ; m < static_cast<size_t>(kNumMotors) ; ++m)
  {
    if (motorIdToRjo_[m] < 0)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
        "[mc_unitree] motor {} ({}) is not in the reference joint order of {}",
        m, motorJointNames[m], robot->name());
    }
  }
  {
    const size_t passive = robot->refJointOrder().size() - static_cast<size_t>(kNumMotors);
    mc_rtc::log::info("[mc_unitree] G1Revo2 mapped {} motors, {} passive joint(s) not commanded",
                      kNumMotors, passive);
  }

  for (size_t i = 0 ; i < robot_->refJointOrder().size() ; i++)
  {
    const int motorId = rjoToMotorId_[i];
    if (motorId < 0) continue;
    cmdOut_.kpOut_[i] = kp_[motorId];
    cmdOut_.kdOut_[i] = kd_[motorId];
  }

  refJointOrderToMCJointId_.resize(robot->refJointOrder().size(), -1);
  for (size_t i = 0 ; i < robot->refJointOrder().size() ; i++)
  {
    const std::string & jname = robot_->refJointOrder()[i];
    auto mcJointId = robot->jointIndexByName(jname);
    if (robot->mbc().q[mcJointId].empty())
      continue;

    refJointOrderToMCJointId_[i] = mcJointId;
    mcJointIdToJointId_[mcJointId] = i;
    // overrite initial q_init_ if stance is set and not provided by the config file
    const int motorId = rjoToMotorId_[i];
    if(motorId >= 0 && robot->stance().count(jname) && !g1_config.has("q_init"))
    {
      q_init_(motorId) = robot->stance().at(jname)[0];
    }
  }

  mode_ = config_param.mode_;

  const std::string &network = config_param.network_;
  const bool is_loopback = (network == "lo" || network.empty());

  if (!network.empty())
  {
    int simulation = (network == "lo") ? 1 : 0;
    mc_rtc::log::info("[mc_unitree] G1Control: Using network setting: {}", network);

    unitree::robot::ChannelFactory::Instance()->Init(simulation, network);
    mc_rtc::log::info("Initialize channel factory.");

    //cmd pub init
    lowcmd_publisher_.reset(
      new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher_->InitChannel();

    handcmd_publisher_left_.reset(
      new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>(TOPIC_BRAINCO_LEFT_CMD));
    handcmd_publisher_left_->InitChannel();

    handcmd_publisher_right_.reset(
      new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::MotorCmds_>(TOPIC_BRAINCO_RIGHT_CMD));
    handcmd_publisher_right_->InitChannel();

    command_writer_ptr_ = unitree::common::CreateRecurrentThreadEx(
      "command_writer", UT_CPU_ID_NONE, 2000, &G1Revo2Control::LowCommandWriter, this);
  
    //state sub init
    lowstate_subscriber_.reset(
      new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber_->InitChannel(
      std::bind(&G1Revo2Control::LowStateHandler, this, std::placeholders::_1),
      1);

    handstate_subscriber_left_.reset(
      new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>(TOPIC_BRAINCO_LEFT_STATE));
    handstate_subscriber_left_->InitChannel(
      std::bind(&G1Revo2Control::HandStateHandler_Left, this, std::placeholders::_1), 1);

    handstate_subscriber_right_.reset(
      new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>(TOPIC_BRAINCO_RIGHT_STATE));
    handstate_subscriber_right_->InitChannel(
      std::bind(&G1Revo2Control::HandStateHandler_Right, this, std::placeholders::_1), 1);
    
#if defined(__ENABLE_RT_PREEMPT__)
    pthread_create(&control_thread_, NULL,
                   [](void* arg) -> void* {
                     auto* ctrl = static_cast<G1Revo2Control::Control*>(arg);
                     ctrl->Control();
                     return nullptr;
                   }, NULL);
#else
    int control_period_us = control_dt_ * 1e6;
    control_thread_ptr_ = unitree::common::CreateRecurrentThreadEx(
      "control", UT_CPU_ID_NONE, control_period_us, &G1Revo2Control::Control,
      this);
#endif
    
    int report_period_us = report_dt_ * 1e6;
    report_sensor_ptr_ = unitree::common::CreateRecurrentThreadEx(
      "report_sensor", UT_CPU_ID_NONE, report_period_us,
      &G1Revo2Control::UpdateTables, this, false);
    
    // Initialize tables for console display
    UpdateTables(true);
  }

  running_ = true;
}

G1Revo2Control::~G1Revo2Control()
{
#if defined(__ENABLE_RT_PREEMPT__)
  pthread_join(control_thread_, NULL);
#endif
}

////
// WAITING BEFORE LAUNCHING CONTROLLER
////

// Wait for Enter key press
void waiting(G1Revo2Control *controller)
{
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(1000ms);
  if(controller->calibrationOffered())
  {
    mc_rtc::log::info("[calib] Robot hanging? Type c + Enter to run the actuator friction identification, Enter to skip");
    std::string line;
    std::getline(std::cin, line);
    if(!line.empty() && (line[0] == 'c' || line[0] == 'C'))
    {
      controller->requestCalibration();
      while(!controller->calibrationDone()) { std::this_thread::sleep_for(200ms); }
      controller->calibrationApplyPrompt();
      mc_rtc::log::info("[calib] Done. Press Enter when the robot is ready for the next phase");
      std::cin.get();
    }
  }
  else
  {
    std::cin.get();
  }
  controller->endWaiting();
}

void G1Revo2Control::endWaiting()
{
  if (status_ == STATUS_WAITING_AIR)
  {
    status_ = STATUS_WAITING_GRD;
    std::thread wait_thread(waiting, this);
    wait_thread.detach();
  }
  else if (status_ == STATUS_WAITING_GRD)
  {
    time_run_ = -control_dt_;
    status_ = STATUS_GAIN_TRANSITION;
  }
}

void G1Revo2Control::LowCommandWriter()
{
  unitree_hg::msg::dds_::LowCmd_ dds_low_command{};
  dds_low_command.mode_pr() = 0;
  dds_low_command.mode_machine() = mode_machine_;

  unitree_go::msg::dds_::MotorCmds_ dds_hand_cmd_left{};
  dds_hand_cmd_left.cmds().resize(6);
  unitree_go::msg::dds_::MotorCmds_ dds_hand_cmd_right{};
  dds_hand_cmd_right.cmds().resize(6);
  
  const std::shared_ptr<const MotorCommand> mc_tmp_ptr =
    motor_command_buffer_.GetData();
  if (mc_tmp_ptr)
  {
    //g1 joints
    for (int i = 0; i < 29; ++i) 
    {
      dds_low_command.motor_cmd().at(i).mode() = 1;
      dds_low_command.motor_cmd().at(i).tau() = mc_tmp_ptr->tau_ff.at(i);
      dds_low_command.motor_cmd().at(i).q() = mc_tmp_ptr->q_ref.at(i);
      dds_low_command.motor_cmd().at(i).dq() = mc_tmp_ptr->dq_ref.at(i);
      dds_low_command.motor_cmd().at(i).kp() = mc_tmp_ptr->kp.at(i);
      dds_low_command.motor_cmd().at(i).kd() = mc_tmp_ptr->kd.at(i);
    }
    dds_low_command.crc() = Crc32Core((uint32_t *)&dds_low_command,
                                      (sizeof(dds_low_command) >> 2) - 1);
    lowcmd_publisher_->Write(dds_low_command);

    //left revo2
    for (int i = 0; i < 6; ++i)
    {
      dds_hand_cmd_left.cmds()[i].q()  = rad_to_norm(29+i, mc_tmp_ptr->q_ref.at(29+i));
      dds_hand_cmd_left.cmds()[i].dq() = 1.0f;  //(from brainco revo2 docu): keep at max speed
    }
    handcmd_publisher_left_->Write(dds_hand_cmd_left);

    //right revo2
    for (int i = 0; i < 6; ++i)
    {
      dds_hand_cmd_right.cmds()[i].q()  = rad_to_norm(35+i, mc_tmp_ptr->q_ref.at(35+i));
      dds_hand_cmd_right.cmds()[i].dq() = 1.0f;
    }
    handcmd_publisher_right_->Write(dds_hand_cmd_right);
  }
}

void G1Revo2Control::LowStateHandler(const void *message)
{
  // mc_rtc::log::info("[mc_unitree] LowStateHandler called");
  //for g1 msg
  unitree_hg::msg::dds_::LowState_ low_state =
    *(unitree_hg::msg::dds_::LowState_ *)message;

  if (low_state.crc() != Crc32Core((uint32_t *)&low_state,
      (sizeof(unitree_hg::msg::dds_::LowState_) >> 2) - 1)) {
    mc_rtc::log::error("[mc_unitree] LowState CRC error on G1 — packet dropped");
    return;
  }

  // update mode_machine_ from robot state
  if (mode_machine_ != low_state.mode_machine())
    mode_machine_ = low_state.mode_machine();

  // g1_state_buffer_.SetData(low_state); //store raw dds messages for the G1 joints
  RecordMotorState(low_state);
  RecordBaseState(low_state);
}

void G1Revo2Control::HandStateHandler_Left(const void *message) 
{
  unitree_go::msg::dds_::MotorStates_ state =
    *(const unitree_go::msg::dds_::MotorStates_ *)message;
  left_hand_state_buffer_.SetData(state);
}

void G1Revo2Control::HandStateHandler_Right(const void *message) 
{
  unitree_go::msg::dds_::MotorStates_ state =
    *(const unitree_go::msg::dds_::MotorStates_ *)message;
  right_hand_state_buffer_.SetData(state);
}

//PS. not used in G1Revo2Control bc g1 and revo2 motor joints are sent in different dds msg formats => it is directly integrated in G1Revo2Control::Control()
void G1Revo2Control::RecordMotorState(const unitree_hg::msg::dds_::LowState_ &msg)
{
  MotorState ms_tmp;
  for (int i = 0; i < 29; ++i)
  {
    ms_tmp.q.at(i) = msg.motor_state()[i].q();
    ms_tmp.dq.at(i) = msg.motor_state()[i].dq();
    ms_tmp.tau.at(i) = msg.motor_state()[i].tau_est();
  }
  motor_state_buffer_.SetData(ms_tmp);
}

void G1Revo2Control::RecordBaseState(const unitree_hg::msg::dds_::LowState_ &msg)
{
  BaseState bs_tmp;
  bs_tmp.omega = msg.imu_state().gyroscope();
  bs_tmp.rpy = msg.imu_state().rpy();
  bs_tmp.quat = msg.imu_state().quaternion();
  bs_tmp.acc = msg.imu_state().accelerometer();
  base_state_buffer_.SetData(bs_tmp);
}

void G1Revo2Control::ReportSensors()
{
  const std::shared_ptr<const BaseState> bs_tmp_ptr =
    base_state_buffer_.GetData();
  const std::shared_ptr<const MotorState> ms_tmp_ptr =
    motor_state_buffer_.GetData();
  if (bs_tmp_ptr)
  {
    mc_rtc::log::info("Base RPY: [{:.4f}, {:.4f}, {:.4f}]",
                      bs_tmp_ptr->rpy.at(0),
                      bs_tmp_ptr->rpy.at(1),
                      bs_tmp_ptr->rpy.at(2));

    mc_rtc::log::info("Gyro: [{:.4f}, {:.4f}, {:.4f}]",
                      bs_tmp_ptr->omega.at(0),
                      bs_tmp_ptr->omega.at(1),
                      bs_tmp_ptr->omega.at(2));
  }
  if (ms_tmp_ptr)
  {
    // report in motor order
    mc_rtc::log::info("Joint positions: [");
    for (size_t m = 0; m < static_cast<size_t>(kNumMotors); ++m)
    {
      mc_rtc::log::info("{:.4f}, ", ms_tmp_ptr->q.at(m));
    }
    mc_rtc::log::info("]");
    mc_rtc::log::info("Joint velocities: [");
    for (size_t m = 0; m < static_cast<size_t>(kNumMotors); ++m)
    {
      mc_rtc::log::info("{:.4f}, ", ms_tmp_ptr->dq.at(m));
    }
    mc_rtc::log::info("]");
  }
}

void G1Revo2Control::UpdateTables(bool init)
{
  if(status_ != prev_status_)
  {
    prev_status_ = status_;
    switch (status_)
    {
    case STATUS_INIT:
      mc_rtc::log::info("    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
      mc_rtc::log::info("    ┃      Initialization      ┃");
      mc_rtc::log::info("    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
      break;
    case STATUS_WAITING_AIR:
      mc_rtc::log::info("    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
      mc_rtc::log::info("    ┃    Waiting in the air    ┃");
      mc_rtc::log::info("    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
      break;
    case STATUS_WAITING_GRD:
      mc_rtc::log::info("    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
      mc_rtc::log::info("    ┃   Waiting on the ground  ┃");
      mc_rtc::log::info("    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
      break;
    case STATUS_GAIN_TRANSITION:
      mc_rtc::log::info("    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
      mc_rtc::log::info("    ┃   PD Gains Transition    ┃");
      mc_rtc::log::info("    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
      break;
    case STATUS_RUN:
      mc_rtc::log::info("    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
      mc_rtc::log::info("    ┃    Running Controller    ┃");
      mc_rtc::log::info("    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
      break;
    case STATUS_DAMPING:
      mc_rtc::log::info("    ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
      mc_rtc::log::info("    ┃    Emergency Damping!    ┃");
      mc_rtc::log::info("    ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
      break;
    }
  }
}

void G1Revo2Control::Control()
{
#if defined(__ENABLE_RT_PREEMPT__)
  if (set_sched_prio(RT_PRIO_MAX-2, TASK_PERIOD) == -1)
  {
    perror("set_sched_prio failed on MCControlUnitree2.\n");
    return;
  }
#endif

  MotorCommand motor_command_tmp;
  const bool is_loopback = (config_.network_ == "lo" || config_.network_.empty());

  // q_pos and q_vel declared here so they are accessible throughout the function
  Vector41 q_pos = Vector41::Zero();
  Vector41 q_vel = Vector41::Zero();

  if (!is_loopback)
  {
    const auto g1_ptr = motor_state_buffer_.GetData();
    const std::shared_ptr<const BaseState> bs_tmp_ptr = base_state_buffer_.GetData();

    if (!g1_ptr || !bs_tmp_ptr)
    {
      if (!g1_ptr) mc_rtc::log::warning("[mc_unitree] No G1 state data. Skipping this iteration...");
      if (!bs_tmp_ptr) mc_rtc::log::warning("[mc_unitree] No base state data. Skipping this iteration...");
      return;
    }

    const auto left_hand_state_ptr = left_hand_state_buffer_.GetData();
    const auto right_hand_state_ptr = right_hand_state_buffer_.GetData();
    if (!left_hand_state_ptr || !right_hand_state_ptr)
    {
      mc_rtc::log::warning("[mc_unitree] No Revo2 state data. Waiting...");
      // keep hand joints at 0, don't return — let G1 joints still run
    }
    else
    {
      // q_pos/q_vel are motor-indexed & stateIn_ is refJointOrder-indexed
      for (int i = 0; i < 6; i++)
      {
        const int lMotor = 29 + i, rMotor = 35 + i;
        const int lRjo = motorIdToRjo_[lMotor], rRjo = motorIdToRjo_[rMotor];

        stateIn_.qIn_[lRjo] = norm_to_rad(lMotor, left_hand_state_ptr->states()[i].q());
        stateIn_.dqIn_[lRjo] = left_hand_state_ptr->states()[i].dq();
        stateIn_.tauIn_[lRjo] = left_hand_state_ptr->states()[i].tau_est();

        stateIn_.qIn_[rRjo] = norm_to_rad(lMotor, right_hand_state_ptr->states()[i].q());
        stateIn_.dqIn_[rRjo] = right_hand_state_ptr->states()[i].dq();
        stateIn_.tauIn_[rRjo] = right_hand_state_ptr->states()[i].tau_est();

        q_pos[lMotor] = stateIn_.qIn_[lRjo];
        q_pos[rMotor] = stateIn_.qIn_[rRjo];
        q_vel[lMotor] = stateIn_.dqIn_[lRjo];
        q_vel[rMotor] = stateIn_.dqIn_[rRjo];
      }
    }

    time_ += control_dt_;

    //for g1 joints
    for (size_t i = 0; i < 29; i++)
    {
      const int rjo = motorIdToRjo_[i];
      stateIn_.qIn_[rjo] = g1_ptr->q.at(i);
      stateIn_.dqIn_[rjo] = g1_ptr->dq.at(i);
      stateIn_.tauIn_[rjo] = g1_ptr->tau.at(i);
      q_pos[i] = g1_ptr->q.at(i);
      q_vel[i] = g1_ptr->dq.at(i);
    }

    //for imu measurements
    for (int i = 0; i < 3; i++)
    {
      stateIn_.rpyIn_[i] = bs_tmp_ptr->rpy.at(i);
      stateIn_.accIn_[i] = bs_tmp_ptr->acc.at(i);
      stateIn_.rateIn_[i] = bs_tmp_ptr->omega.at(i);
    }
    for (int i = 0; i < 4; i++)
    {
      stateIn_.quatIn_.coeffs()[i] = bs_tmp_ptr->quat.at(i);
    }
  }
  else
  {
    // loopback: stateIn_ already populated via loopbackState() or setInitialState()
    time_ += control_dt_;
    for (size_t i = 0; i < robot_->refJointOrder().size(); i++)
    {
      const int motorId = rjoToMotorId_[i];
      if (motorId < 0) continue;
      q_pos[motorId] = static_cast<float>(stateIn_.qIn_[i]);
      q_vel[motorId] = static_cast<float>(stateIn_.dqIn_[i]);
    }
  }

  // Check if joints are too close from position limits
  const bool lim_lower = ((q_pos - q_lim_lower_).array() < 0.0).any();
  const bool lim_upper = ((q_pos - q_lim_upper_).array() > 0.0).any();
  const bool lim_velocity_lower = ((q_vel - q_dot_lim_lower_).array() < 0.0).any();
  const bool lim_velocity_upper = ((q_vel - q_dot_lim_upper_).array() > 0.0).any();
  
  // Switch to waiting after initialization
  if ((status_ == STATUS_INIT) && (time_ > init_duration_))
  {
    if (is_loopback && !mc_controller_->calibration())
    {
      // skip waiting states in loopback, go straight to run (unless --calib: dry-run the sweep)
      time_run_ = -control_dt_;
      status_ = STATUS_GAIN_TRANSITION;
    }
    else
    {
      status_ = STATUS_WAITING_AIR;
      std::thread wait_thread(waiting, this);
      wait_thread.detach();
    }
  }
  
  switch (status_)
  {
  case STATUS_RUN:
  {
    // If limits are breached, go to damping mode
    if (!is_loopback && (lim_lower || lim_upper || lim_velocity_lower || lim_velocity_upper))
    {
      // q_pos/q_vel and the limit vectors are motor-indexed 
      const int n = kNumMotors;
      if (lim_lower)
      {
        mc_rtc::log::error("[mc_unitree] Joint lower position limit breached!");
        for(int i = 0; i < n; ++i)
        {
          if(q_pos[i] < q_lim_lower_[i])
          {
            mc_rtc::log::error("  - {}: pos = {}, lower limit = {}", 
                              motorJointNames[i], q_pos[i], q_lim_lower_[i]);
          }
        }
      }

      if (lim_upper)
      {
        mc_rtc::log::error("[mc_unitree] Joint upper position limit breached!");
        for(int i = 0; i < n; ++i)
        {
          if(q_pos[i] > q_lim_upper_[i])
          {
            mc_rtc::log::error("  - {}: pos = {}, upper limit = {}", 
                              motorJointNames[i], q_pos[i], q_lim_upper_[i]);
          }
        }
      }

      if (lim_velocity_lower)
      {
        mc_rtc::log::error("[mc_unitree] Joint lower velocity limit breached!");
        for(int i = 0; i < n; ++i)
        {
          if(q_vel[i] < q_dot_lim_lower_[i])
          {
            mc_rtc::log::error("  - {}: vel = {}, lower vel limit = {}", 
                              motorJointNames[i], q_vel[i], q_dot_lim_lower_[i]);
          }
        }
      }

      if (lim_velocity_upper)
      {
        mc_rtc::log::error("[mc_unitree] Joint upper velocity limit breached!");
        for(int i = 0; i < n; ++i)
        {
          if(q_vel[i] > q_dot_lim_upper_[i])
          {
            mc_rtc::log::error("  - {}: vel = {}, upper vel limit = {}", 
                              motorJointNames[i], q_vel[i], q_dot_lim_upper_[i]);
          }
        }
      }

      status_ = STATUS_DAMPING;
    }

    auto &datastore = mc_controller_->controller().controller().datastore();
    if (datastore.has("ControlMode"))
    {
      setControlMode(datastore.get<std::string>("ControlMode"));
    }

    time_run_ += control_dt_;

    mc_controller_->run(stateIn_, cmdOut_);
    
    // Send commands to the robot
    if(mode_ == ControlMode::Position)
    {
      for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
      {
        const int motorId = rjoToMotorId_[i];
      if (motorId < 0) continue; // passive (mimic) joint: no motor to command
        motor_command_tmp.kp.at(motorId) = cmdOut_.kpOut_[i];
        motor_command_tmp.kd.at(motorId) = cmdOut_.kdOut_[i];
        motor_command_tmp.q_ref.at(motorId) = cmdOut_.qOut_[i];
        motor_command_tmp.dq_ref.at(motorId) = cmdOut_.dqOut_[i];
        motor_command_tmp.tau_ff.at(motorId) = 0.f;
      }
      break;
    }
    else if(mode_ == ControlMode::Torque)
    {
      for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
      {
        const int motorId = rjoToMotorId_[i];
      if (motorId < 0) continue; // passive (mimic) joint: no motor to command
        motor_command_tmp.kp.at(motorId) = 0.f;
        motor_command_tmp.kd.at(motorId) = 0.f;
        motor_command_tmp.tau_ff.at(motorId) = cmdOut_.tauOut_[i];
      }
      break;
    }
    mc_rtc::log::error("[mc_unitree] Unknown control mode!");
    status_ = STATUS_DAMPING;
    break;
  }
  
  case STATUS_WAITING_AIR:
  {
    // --calib: the sweep overrides the hold targets of the joints it drives
    const bool calibActive = calibrationStep(q_pos, q_vel);
    for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
    {
      const int motorId = rjoToMotorId_[i];
      if (motorId < 0) continue; // passive (mimic) joint: no motor to command
      motor_command_tmp.kp.at(motorId) = kp_wait_(motorId);
      motor_command_tmp.kd.at(motorId) = kd_wait_(motorId);
      motor_command_tmp.q_ref.at(motorId) = calibActive ? calibQRef_(motorId) : q_init_(motorId);
      motor_command_tmp.dq_ref.at(motorId) = 0.f;
      motor_command_tmp.tau_ff.at(motorId) = 0.f;
    }
    if(calibActive)
    {
      if(is_loopback)
      {
        // no robot: echo our own targets as the measured state (dq by finite difference)
        for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
        {
          const int motorId = rjoToMotorId_[i];
          if (motorId < 0) continue;
          const double qn = static_cast<double>(calibQRef_(motorId));
          stateIn_.dqIn_[i] = (qn - stateIn_.qIn_[i]) / control_dt_;
          stateIn_.qIn_[i] = qn;
          stateIn_.tauIn_[i] = 0.0;
        }
      }
      // Run mc_rtc passively: observers log the sweep.
      mc_controller_->runPassive(stateIn_);
    }
    break;
  }
  
  case STATUS_WAITING_GRD:
  {
    for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
    {
      const int motorId = rjoToMotorId_[i];
      if (motorId < 0) continue; // passive (mimic) joint: no motor to command
      motor_command_tmp.kp.at(motorId) = kp_wait_(motorId);
      motor_command_tmp.kd.at(motorId) = kd_wait_(motorId);
      motor_command_tmp.q_ref.at(motorId) = q_init_(motorId);
      motor_command_tmp.dq_ref.at(motorId) = 0.f;
      motor_command_tmp.tau_ff.at(motorId) = 0.f;
    }
    break;
  }
  
  case STATUS_GAIN_TRANSITION:
  {
    time_run_ += control_dt_;
    float alpha = time_run_ / interp_duration_;
    for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
    {
      const int motorId = rjoToMotorId_[i];
      if (motorId < 0) continue; // passive (mimic) joint: no motor to command
      motor_command_tmp.kp.at(motorId) = kp_wait_(motorId) * (1 - alpha) + kp_(motorId) * alpha;
      motor_command_tmp.kd.at(motorId) = kd_wait_(motorId) * (1 - alpha) + kd_(motorId) * alpha;
      motor_command_tmp.q_ref.at(motorId) = q_init_(motorId);
      motor_command_tmp.dq_ref.at(motorId) = 0.f;
      motor_command_tmp.tau_ff.at(motorId) = 0.f;
    }
    if (time_run_ >= interp_duration_)
    {
      time_run_ = -control_dt_;
      status_ = STATUS_RUN;
    }
    break;
  }
  
  case STATUS_INIT:
  {
    float ratio = std::clamp(time_, 0.f, init_duration_) / init_duration_;
    for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
    {
      const int motorId = rjoToMotorId_[i];
      if (motorId < 0) continue; // passive (mimic) joint: no motor to command
      motor_command_tmp.kp.at(motorId) = kp_wait_(motorId);
      motor_command_tmp.kd.at(motorId) = kd_wait_(motorId);
      motor_command_tmp.dq_ref.at(motorId) = 0.f;
      motor_command_tmp.tau_ff.at(motorId) = 0.f;
      // interpolate from current q to q_init
      float q_current = static_cast<float>(stateIn_.qIn_[i]);
      float q_des = (q_init_(motorId) - q_current) * ratio + q_current;
      motor_command_tmp.q_ref.at(motorId) = q_des;
    }
    break;
  }
  
  default:
  { // case STATUS_DAMPING:
    for (size_t i = 0 ; i < robot_->refJointOrder().size() ; i++)
    {
      const int motorId = rjoToMotorId_[i];
      if (motorId < 0) continue; // passive (mimic) joint: no motor to command
      motor_command_tmp.kp.at(motorId) = 0.f;
      motor_command_tmp.kd.at(motorId) = kd_(motorId);
      motor_command_tmp.q_ref.at(motorId) = static_cast<float>(stateIn_.qIn_[i]);
      motor_command_tmp.dq_ref.at(motorId) = 0.f;
      motor_command_tmp.tau_ff.at(motorId) = 0.f;
    }
  }
  }

  // Write to command buffer
  motor_command_buffer_.SetData(motor_command_tmp);
  
  // Log estimated torques
  for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
  {
    const int motorId = rjoToMotorId_[i];
    if (motorId < 0) continue; // passive (mimic) joint: no motor to command
    tau_des_[motorId] =
      motor_command_tmp.kp.at(motorId) *
      (motor_command_tmp.q_ref.at(motorId) - static_cast<float>(stateIn_.qIn_[i])) +
      motor_command_tmp.kd.at(motorId) *
      (motor_command_tmp.dq_ref.at(motorId) - static_cast<float>(stateIn_.dqIn_[i])) +
      motor_command_tmp.tau_ff.at(motorId);
  }

  // In loopback mode, feed commands back as state (not during the calibration sweep,
  // which echoes its own targets: cmdOut_ is not filled while mc_rtc runs passively)
  if (is_loopback && !calibRunning_)
  {
    loopbackState(cmdOut_);
  }
}

/**
 * @brief Set initial state values for simulation
 */
void G1Revo2Control::setInitialState(const std::map<std::string, std::vector<double>> & stance)
{
  for (size_t i = 0 ; i < robot_->refJointOrder().size() ; i++)
  {
    const std::string & jname = robot_->refJointOrder()[i];
    auto mcJointId = robot_->jointIndexByName(jname);
    auto jointId = mcJointIdToJointId(mcJointId);
    
    if(stance.count(jname))
    {
      stateIn_.qIn_[jointId] = stance.at(jname)[0];
    }
    else
    {
      stateIn_.qIn_[jointId] = 0.0;
    }
    stateIn_.dqIn_[jointId] = 0.0;
    stateIn_.tauIn_[jointId] = 0.0;
  }
  
  stateIn_.rpyIn_.setZero();
  stateIn_.quatIn_.setIdentity();
  stateIn_.accIn_.setZero();
  stateIn_.rateIn_.setZero();
}

/**
 * @brief Loop back the value of "data" to "state"
 */
void G1Revo2Control::loopbackState(const G1Revo2CommandData & data)
{
  for (size_t i = 0 ; i < robot_->refJointOrder().size() ; i++)
  {
    stateIn_.qIn_[i] = data.qOut_[i];
    stateIn_.dqIn_[i] = data.dqOut_[i];
    stateIn_.tauIn_[i] = data.tauOut_[i];
    
    if (i < robot_->qu().size() && !robot_->qu()[i].empty() && stateIn_.qIn_[i] > robot_->qu()[i][0])
      stateIn_.qIn_[i] = robot_->qu()[i][0];
    else if (i < robot_->ql().size() && !robot_->ql()[i].empty() && stateIn_.qIn_[i] < robot_->ql()[i][0])
      stateIn_.qIn_[i] = robot_->ql()[i][0];
  }
}

void G1Revo2Control::setControlMode(const std::string &mode) {
  if (mode.compare("Position") == 0) {
    mode_ = ControlMode::Position;
    return;
  }
  else if (mode.compare("Torque") == 0) {
   mode_ = ControlMode::Torque;
   return;
  }
  else {
    mc_rtc::log::error("{} ControlMode not supported", mode);
  }
}


float G1Revo2Control::norm_to_rad(int joint_idx, float norm) const {
    if (joint_idx < 29) return norm;
    float lower = q_lim_lower_[joint_idx];
    float upper = q_lim_upper_[joint_idx];
    return lower + norm * (upper - lower);
}

float G1Revo2Control::rad_to_norm(int joint_idx, float rad) const {
    if (joint_idx < 29) return rad;
    float lower = q_lim_lower_[joint_idx];
    float upper = q_lim_upper_[joint_idx];
    return (rad - lower) / (upper - lower);
}

// Actuator friction identification fct()

bool mc_unitree::G1Revo2Control::calibrationOffered() const
{
  return mc_controller_->calibration() && status_ == STATUS_WAITING_AIR;
}

void mc_unitree::G1Revo2Control::computeGravityTorques(Vector41 & g)
{
  const auto & mb = robot_->mb();
  rbd::MultiBodyConfig mbc = robot_->mbc(); // shape and passive joints from the control robot
  for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
  {
    const int mcJointId = refJointOrderToMCJointId_[i];
    if (mcJointId < 0 || static_cast<size_t>(mcJointId) >= mbc.q.size()) continue;
    if (mbc.q[static_cast<size_t>(mcJointId)].size() == 1) { mbc.q[static_cast<size_t>(mcJointId)][0] = stateIn_.qIn_[i]; }
  }
  // Root at identity: gravity is expressed in the root frame below.
  if (!mbc.q.empty() && mbc.q[0].size() == 7) { mbc.q[0] = {1., 0., 0., 0., 0., 0., 0.}; }
  for (auto & a : mbc.alpha) { std::fill(a.begin(), a.end(), 0.0); }
  for (auto & a : mbc.alphaD) { std::fill(a.begin(), a.end(), 0.0); }
  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);
  // RBDyn's mbc.gravity is the upward specific force the base "feels" (default 0,0,9.81):
  // exactly what the IMU accelerometer reads at rest. Rotate the IMU reading into the
  // root frame so a tilted hang does not bias g(q). Loopback (no IMU data) -> upright.
  Eigen::Vector3d gravity(0., 0., 9.81);
  const double an = stateIn_.accIn_.norm();
  if (an > 8.0 && an < 11.5 && !robot_->bodySensors().empty())
  {
    const auto & bs = robot_->bodySensor();
    const auto & X_0_p = mbc.bodyPosW[static_cast<size_t>(mb.bodyIndexByName(bs.parentBody()))];
    const Eigen::Matrix3d E_0_s = bs.X_b_s().rotation() * X_0_p.rotation(); // root -> sensor
    gravity = E_0_s.transpose() * (stateIn_.accIn_ * (9.81 / an));
  }
  mbc.gravity = gravity;
  rbd::ForwardDynamics fd(mb);
  fd.computeC(mb, mbc); // with zero velocity, C() is the gravity torque vector
  const Eigen::VectorXd & C = fd.C();
  g.setZero();
  for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
  {
    const int motorId = rjoToMotorId_[i];
    const int mcJointId = refJointOrderToMCJointId_[i];
    if (motorId < 0 || mcJointId < 0) continue;
    g(motorId) = static_cast<float>(C(mb.jointPosInDof(mcJointId)));
  }
}

bool mc_unitree::G1Revo2Control::calibrationStep(const Vector41 & q_pos, const Vector41 & q_vel)
{
  if (!calibRequested_ || calibDone_) { return false; }
  if (!calibRunning_)
  {
    calib_ = std::make_unique<ActuatorCalibration>(static_cast<double>(control_dt_), motorJointNames, q_lim_lower_, q_lim_upper_, q_init_);
    Vector41 qNow = q_pos;
    if (!calib_->start(ActuatorCalibration::filterTable(ActuatorCalibration::defaultTable(), mc_controller_->calibrationSet()), qNow))
    {
      mc_rtc::log::error("[calib] could not build the sweep, aborting");
      calibDone_ = true;
      return false;
    }
    calibRunning_ = true;
    mc_rtc::log::info("[calib] sweep started ({}): {:.0f} s planned, joints are driven with kp_wait/kd_wait", mc_controller_->calibrationSet(), calib_->planned());
    // The GUI's "Robots" entry shows mc_rtc's commanded (output) robot, which does not
    // move during the sweep. Publish the REAL robot under its own category so the
    // client shows the sweep (mc-rtc-magnum hides robots only under "Robots").
    {
      auto & ctl = mc_controller_->controller().controller();
      ctl.gui()->addElement({"Calibration"},
                            mc_rtc::gui::Robot("real robot (calibration)",
                                               [&ctl]() -> const mc_rbdyn::Robot & { return ctl.realRobot(); }));
    }
    // mc_rtc runs passively during the sweep with the feet in the air: its balance code
    // logs a ZMP error every cycle. Mute error/warning output until the sweep is over so
    // the calibration progress and report stay readable (info/success stay on).
    calibPrevErrLevel_ = mc_rtc::log::details::cerr().level();
    mc_rtc::log::details::cerr().set_level(spdlog::level::off);
  }
  Vector41 tau = Vector41::Zero(), g = Vector41::Zero();
  for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
  {
    const int motorId = rjoToMotorId_[i];
    if (motorId < 0) continue;
    tau(motorId) = static_cast<float>(stateIn_.tauIn_[i]);
  }
  computeGravityTorques(g);
  calib_->update(q_pos, q_vel, tau, g);
  calibQRef_ = calib_->qRef();
  if (!calib_->active())
  {
    mc_controller_->controller().controller().gui()->removeCategory({"Calibration"});
    mc_rtc::log::details::cerr().set_level(calibPrevErrLevel_);
    calib_->fit();
    calibRunning_ = false;
    calibDone_ = true;
    return false;
  }
  return true;
}

void mc_unitree::G1Revo2Control::calibrationApplyPrompt()
{
  if (!calib_) { return; }
  mc_rtc::log::info("[calib] ---------------- actuator friction identification ----------------\n{}", calib_->report());
  if (!calib_->identifiedAny())
  {
    mc_rtc::log::warning("[calib] no joint identifiable (expected in loopback: torques are zero). Nothing written.");
    return;
  }
  const std::string robotName = mc_controller_->controller().robot().name();
  const std::string path = (std::filesystem::path(mc_rtc::user_config_directory_path("observers")) / "ExternalForcesObserver" / (robotName + ".yaml")).string();
  std::cout << calib_->yaml();
  mc_rtc::log::info("[calib] Write this friction_model to {} (loaded by every controller's ExternalForcesObserver on this robot)? [y/N] ", path);
  std::string line;
  std::getline(std::cin, line);
  if (line.empty() || (line[0] != 'y' && line[0] != 'Y'))
  {
    mc_rtc::log::info("[calib] not written");
    return;
  }
  if (!calib_->write(path))
  {
    mc_rtc::log::error("[calib] could not write {}", path);
    return;
  }
  mc_rtc::log::success("[calib] written {}", path);
  auto & ds = mc_controller_->controller().controller().datastore();
  if (ds.has("EF_Estimator::reloadFrictionModel"))
  {
    if (ds.call<bool>("EF_Estimator::reloadFrictionModel"))
    {
      mc_rtc::log::success("[calib] observer reloaded the friction model");
    }
    else
    {
      mc_rtc::log::warning("[calib] observer could not reload the friction model; check the file, it is used from the next start");
    }
  }
  else
  {
    mc_rtc::log::warning("[calib] observer reload call not found; the model is used from the next start");
  }
}
