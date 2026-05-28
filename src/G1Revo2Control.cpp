#include <mc_control/mc_global_controller.h>
#include <mc_rtc/logging.h>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>
#include "G1Revo2Control.h"

// ===============================================================================================================
// TODO.
// 1. change dds_low_command_revo2.mode_pr() and dds_low_command_revo2.mode_machine() to adapt to revo2
// msg format (unitree go2 messages, base code from how it was done in the H1 controller), do the same for 
// dds_low_command_revo2.motor_cmd().at(i).mode()
// ===============================================================================================================

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
  for (size_t i = 0 ; i < robot_->refJointOrder().size() ; i++)
  {
    cmdOut_.kpOut_[i] = kp_[i];
    cmdOut_.kdOut_[i] = kd_[i];
  }
  
  refJointOrderToMCJointId_.resize(kNumMotors, -1);
  for (size_t i = 0 ; i < robot->refJointOrder().size() ; i++)
  {
    const std::string & jname = robot_->refJointOrder()[i];
    auto mcJointId = robot->jointIndexByName(jname);
    if (robot->mbc().q[mcJointId].empty())
      continue;
    
    refJointOrderToMCJointId_[i] = mcJointId;
    mcJointIdToJointId_[mcJointId] = i;
    /* Overrite initial q_init_ if stance is set and not provided by the config file */
    if(robot->stance().count(jname) && !g1_config.has("q_init"))
    {
      q_init_(i) = robot->stance().at(jname)[0];
    }
  }

  mode_ = config_param.mode_;

  const std::string &network = config_param.network_;
  const bool is_loopback = (network == "lo" || network.empty());

  if (!network.empty())
  {
    int simulation = 1;
    if(network != "lo") simulation = 0;
    mc_rtc::log::info("[mc_unitree] G1Control: Using network setting: {}", network);

    /* Initialize */
    unitree::robot::ChannelFactory::Instance()->Init(simulation, network);
    mc_rtc::log::info("Initialize channel factory.");
    
    lowcmd_publisher_.reset(
      new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher_->InitChannel();
    lowcmd_publisher_revo2.reset(
      new unitree::robot::ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD_REVO2));
    lowcmd_publisher_revo2->InitChannel();
    command_writer_ptr_ = unitree::common::CreateRecurrentThreadEx(
      "command_writer", UT_CPU_ID_NONE, 2000, &G1Revo2Control::LowCommandWriter, this);
  
    lowstate_subscriber_.reset(
      new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber_->InitChannel(
      std::bind(&G1Revo2Control::LowStateHandler, this, std::placeholders::_1),
      1);
    lowstate_subscriber_revo2.reset(
      new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE_REVO2));
    lowstate_subscriber_revo2->InitChannel(
      std::bind(&G1Revo2Control::LowStateHandler_Revo2, this, std::placeholders::_1),
      1);
    
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
  std::cin.get();
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

  unitree_go::msg::dds_::LowCmd_ dds_low_command_revo2{};
  dds_low_command_revo2.level_flag() = 0xFF;
  
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

    //revo2 joints
    for (int i = 0; i < 12; ++i) 
    {
      int idx = 29 + i;
      dds_low_command_revo2.motor_cmd().at(i).mode() = 1;
      dds_low_command_revo2.motor_cmd().at(i).tau() = mc_tmp_ptr->tau_ff.at(idx);
      dds_low_command_revo2.motor_cmd().at(i).q() = rad_to_norm(idx, mc_tmp_ptr->q_ref.at(idx));
      dds_low_command_revo2.motor_cmd().at(i).dq() = rad_to_norm(idx, mc_tmp_ptr->dq_ref.at(idx));
      dds_low_command_revo2.motor_cmd().at(i).kp() = mc_tmp_ptr->kp.at(idx);
      dds_low_command_revo2.motor_cmd().at(i).kd() = mc_tmp_ptr->kd.at(idx);
    }
    dds_low_command_revo2.crc() = Crc32Core((uint32_t *)&dds_low_command_revo2,
                                      (sizeof(dds_low_command_revo2) >> 2) - 1);
    lowcmd_publisher_revo2->Write(dds_low_command_revo2);
  }
}

void G1Revo2Control::LowStateHandler(const void *message)
{
  mc_rtc::log::info("[mc_unitree] LowStateHandler called");
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

void G1Revo2Control::LowStateHandler_Revo2(const void *message_revo2)
{
  mc_rtc::log::info("[mc_unitree] LowStateHandler_Revo2 called");
  //for revo2 msg
  unitree_go::msg::dds_::LowState_ low_state_revo2 =
    *(unitree_go::msg::dds_::LowState_ *)message_revo2;

  if (low_state_revo2.crc() != Crc32Core((uint32_t *)&low_state_revo2,
      (sizeof(unitree_go::msg::dds_::LowState_) >> 2) - 1)) {
    mc_rtc::log::error("[mc_unitree] LowState CRC error on Revo2 — packet dropped");
    return;
  }

  revo2_state_buffer_.SetData(low_state_revo2); //store raw dds messages for the revo2 joints
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
    mc_rtc::log::info("Joint positions: [");
    for (size_t i = 0; i < robot_->refJointOrder().size(); ++i)
    {
      mc_rtc::log::info("{:.4f}, ", ms_tmp_ptr->q.at(jointIdsToMotorIds[i]));
    }
    mc_rtc::log::info("]");
    mc_rtc::log::info("Joint velocities: [");
    for (size_t i = 0; i < robot_->refJointOrder().size(); ++i)
    {
      mc_rtc::log::info("{:.4f}, ", ms_tmp_ptr->dq.at(jointIdsToMotorIds[i]));
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
    const auto revo2_ptr = revo2_state_buffer_.GetData();
    const std::shared_ptr<const BaseState> bs_tmp_ptr = base_state_buffer_.GetData();

    if (!g1_ptr || !revo2_ptr || !bs_tmp_ptr)
    {
      if (!g1_ptr) mc_rtc::log::warning("[mc_unitree] No G1 state data");
      if (!revo2_ptr) mc_rtc::log::warning("[mc_unitree] No Revo2 state data");
      if (!bs_tmp_ptr) mc_rtc::log::warning("[mc_unitree] No base state data");
      return;
    }
    time_ += control_dt_;

    //for g1 joints
    for (size_t i = 0; i < 29; i++)
    {
      stateIn_.qIn_[i] = g1_ptr->q.at(i);
      stateIn_.dqIn_[i] = g1_ptr->dq.at(i);
      stateIn_.tauIn_[i] = g1_ptr->tau.at(i);
      q_pos[i] = g1_ptr->q.at(i);
      q_vel[i] = g1_ptr->dq.at(i);
    }

    //for revo2 joints
    for (int i = 0; i < 12; ++i)
    {
      int idx = 29 + i;
      stateIn_.qIn_[idx] = norm_to_rad(idx, revo2_ptr->motor_state()[i].q());
      stateIn_.dqIn_[idx] = norm_to_rad(idx, revo2_ptr->motor_state()[i].dq());
      stateIn_.tauIn_[idx] = revo2_ptr->motor_state()[i].tau_est();
      q_pos[idx] = stateIn_.qIn_[idx];
      q_vel[idx] = stateIn_.dqIn_[idx];
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
      q_pos[i] = static_cast<float>(stateIn_.qIn_[i]);
      q_vel[i] = static_cast<float>(stateIn_.dqIn_[i]);
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
    if (is_loopback)
    {
      // skip waiting states in loopback, go straight to run
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
      const int n = joint_names_.size();
      if (lim_lower)
      {
        mc_rtc::log::error("[mc_unitree] Joint lower position limit breached!");
        for(int i = 0; i < n; ++i)
        {
          if(q_pos[i] < q_lim_lower_[i])
          {
            mc_rtc::log::error("  - {}: pos = {}, lower limit = {}", 
                              joint_names_[i], q_pos[i], q_lim_lower_[i]);
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
                              joint_names_[i], q_pos[i], q_lim_upper_[i]);
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
                              joint_names_[i], q_vel[i], q_dot_lim_lower_[i]);
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
                              joint_names_[i], q_vel[i], q_dot_lim_upper_[i]);
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
        auto motorId = jointIdsToMotorIds[i];
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
        auto motorId = jointIdsToMotorIds[i];
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
    for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
    {
      auto motorId = jointIdsToMotorIds[i];
      motor_command_tmp.kp.at(motorId) = kp_wait_(i);
      motor_command_tmp.kd.at(motorId) = kd_wait_(i);
      motor_command_tmp.q_ref.at(motorId) = q_init_(i);
      motor_command_tmp.dq_ref.at(motorId) = 0.f;
      motor_command_tmp.tau_ff.at(motorId) = 0.f;
    }
    break;
  }
  
  case STATUS_WAITING_GRD:
  {
    for (size_t i = 0 ; i < robot_->refJointOrder().size() ; ++i)
    {
      auto motorId = jointIdsToMotorIds[i];
      motor_command_tmp.kp.at(motorId) = kp_wait_(i);
      motor_command_tmp.kd.at(motorId) = kd_wait_(i);
      motor_command_tmp.q_ref.at(motorId) = q_init_(i);
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
      auto motorId = jointIdsToMotorIds[i];
      motor_command_tmp.kp.at(motorId) = kp_wait_(i) * (1 - alpha) + kp_(i) * alpha;
      motor_command_tmp.kd.at(motorId) = kd_wait_(i) * (1 - alpha) + kd_(i) * alpha;
      motor_command_tmp.q_ref.at(motorId) = q_init_(i);
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
      auto motorId = jointIdsToMotorIds[i];
      motor_command_tmp.kp.at(motorId) = kp_wait_(i);
      motor_command_tmp.kd.at(motorId) = kd_wait_(i);
      motor_command_tmp.dq_ref.at(motorId) = 0.f;
      motor_command_tmp.tau_ff.at(motorId) = 0.f;
      // interpolate from current q to q_init
      float q_current = static_cast<float>(stateIn_.qIn_[i]);
      float q_des = (q_init_(i) - q_current) * ratio + q_current;
      motor_command_tmp.q_ref.at(motorId) = q_des;
    }
    break;
  }
  
  default:
  { // case STATUS_DAMPING:
    for (size_t i = 0 ; i < robot_->refJointOrder().size() ; i++)
    {
      auto motorId = jointIdsToMotorIds[i];
      motor_command_tmp.kp.at(motorId) = 0.f;
      motor_command_tmp.kd.at(motorId) = kd_(i);
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
    auto motorId = jointIdsToMotorIds[i];
    tau_des_[i] =
      motor_command_tmp.kp.at(motorId) *
      (motor_command_tmp.q_ref.at(motorId) - static_cast<float>(stateIn_.qIn_[i])) +
      motor_command_tmp.kd.at(motorId) *
      (motor_command_tmp.dq_ref.at(motorId) - static_cast<float>(stateIn_.dqIn_[i])) +
      motor_command_tmp.tau_ff.at(motorId);
  }

  // In loopback mode, feed commands back as state
  if (is_loopback)
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