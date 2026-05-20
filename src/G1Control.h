#pragma once

#include <chrono>
#include <string>
#include <thread>
#include <Eigen/Core>
#include <vector>
#include <map>

#include <lib/fort.hpp>

//#define __ENABLE_RT_PREEMPT__

#if defined(__ENABLE_RT_PREEMPT__)
#include <pthread.h>
#include "rtapi.h"
#endif

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>

// located in unitree_sdk2/example/h1/low_level 
#include <base_state.h>
#include <data_buffer.hpp>
#include <motors.hpp>

#include <mc_control/mc_controller.h>
#include "MCControlUnitree2.h"

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

using Vector29 = Eigen::Matrix<float, 29, 1>;
using Vector3 = Eigen::Vector3d;


// ===============================================================================================================
// TODO.
// 1. (#include <h1/lowlevel>) create G1-specific headers in unitree_sdk2/examples/g1/low_level
// 2. (mc_unitree::G1Control) change motor_cmd_(motor_cmd) and motor_state_(motor_state) to
// be used as 29dof msgs (unlike Go2/H1's 20dof format msg)
// 3. (mc_unitree::G1Control::IsWeakMotor) what does this fct mean ?
// ===============================================================================================================

namespace mc_unitree
{
  const std::string ROBOT_NAME = "g1";
  
  const int jointIdsToMotorIds[29] =
    {JointIndex::LeftHipPitch,
     JointIndex::LeftHipRoll,
     JointIndex::LeftHipYaw,
     JointIndex::LeftKnee,
     JointIndex::LeftAnklePitch,
     JointIndex::LeftAnkleRoll,
     JointIndex::RightHipPitch,
     JointIndex::RightHipRoll,
     JointIndex::RightHipYaw,
     JointIndex::RightKnee,
     JointIndex::RightAnklePitch,
     JointIndex::RightAnkleRoll,
     JointIndex::WaistYaw,
     JointIndex::WaistRoll,
     JointIndex::WaistPitch,
     JointIndex::LeftShoulderPitch,
     JointIndex::LeftShoulderRoll,
     JointIndex::LeftShoulderYaw,
     JointIndex::LeftElbow,
     JointIndex::LeftWristRoll,
     JointIndex::LeftWristPitch,
     JointIndex::LeftWristYaw,
     JointIndex::RightShoulderPitch,
     JointIndex::RightShoulderRoll,
     JointIndex::RightShoulderYaw,
     JointIndex::RightElbow,
     JointIndex::RightWristRoll,
     JointIndex::RightWristPitch,
     JointIndex::RightWristYaw,
    };
  
/**
 * @brief Configuration file parameters for mc_unitree
 */
struct G1ConfigParameter
{
  G1ConfigParameter()
  : network_(""), mode_(ControlMode::Position)
  {}
  /* Communication information with a real robot */
  /* Connection network */
  std::string network_;
  /* ControlMode : Position/Velocity/Torque (Velocity is not supported)*/
  ControlMode mode_ = ControlMode::Position;
  
  // Default configuration (from mc_g1)
  Vector29 q_init_{
    -0.312, 0.0, 0.0,  
    0.669, -0.363, 0.0,
    -0.312, 0.0, 0.0,
    0.669, -0.363, 0.0, 
    0.0, 0.0, 0.0,
    0.2,  0.2,  0.0, 
    0.6,
    0.0, 0.0, 0.0,
    0.2, -0.2, 0.0,
    0.6,
    0.0, 0.0,  0.0};          

  // Limits defined by Unitree (from g1_mj_description/xml)
  Vector29 q_lim_lower_{
    -2.5307, -0.5236, -2.7576,
    -0.087267, -0.87267, -0.2618,
    -2.5307, -2.9671, -2.7576,
    -0.087267, -0.87267, -0.2618,
    -2.618, -0.52, -0.52,
    -3.0892, -1.5882, -2.618,
    -1.0472,
    -1.97222, -1.61443, -1.61443,
    -3.0892, -2.2515, -2.618,
    -1.0472,
    -1.97222, -1.61443, -1.61443};                 
  Vector29 q_lim_upper_{
    2.8798, 2.9671, 2.7576, 
    2.8798, 0.5236, 0.2618,
    2.8798, 0.5236, 2.7576, 
    2.8798, 0.5236, 0.2618,
    2.618, 0.52, 0.52,
    2.6704, 2.2515, 2.618,
    2.0944,
    1.97222, 1.61443, 1.61443,
    2.6704, 1.5882, 2.618,
    2.0944,
    1.97222, 1.61443, 1.61443};                 

  // from g1_description/urdf
  Vector29 qdot_lim_{    
    32.0, 32.0, 32.0,
    20.0, 30.0, 30.0,
    32.0, 32.0, 32.0,
    20.0, 30.0, 30.0,
    32.0, 30.0, 30.0,
    37.0, 37.0, 37.0,
    37.0
    37, 22, 22,
    37.0, 37.0, 37.0,
    37.0
    37, 22, 22};               
  
  // from g1_mj_description/pdgains
  Vector29 kp_{
    200.0, 100.0, 100.0,
    210.0, 150.0, 150.0,
    200.0, 100.0, 100.0,
    210.0, 150.0, 150.0,
    300.0, 300.0, 300.0,
    100.0, 100.0, 50.0,
    50.0,
    2.0, 2.0, 2.0,
    100.0, 100.0, 50.0,
    50.0,
    2.0, 2.0, 2.0};
  Vector29 kd_{
    4.0, 2.0, 2.0,
    6.0, 4.0, 3.0,
    3.0, 2.0, 2.0,
    6.0, 3.0, 3.0,
    10.0, 3.0, 3.0,
    2.0, 2.0, 2.0,
    2.0,
    2.0, 2.0, 2.0,
    2.0, 2.0, 2.0,
    2.0,
    2.0, 2.0, 2.0};
  Vector29 kp_stand_{
    200.0, 100.0, 100.0,
    210.0, 150.0, 150.0,
    200.0, 100.0, 100.0,
    210.0, 150.0, 150.0,
    300.0, 300.0, 300.0,
    100.0, 100.0, 50.0,
    50.0,
    2.0, 2.0, 2.0,
    100.0, 100.0, 50.0,
    50.0,
    2.0, 2.0, 2.0};             
  Vector29 kd_stand_{
    4.0, 2.0, 2.0,
    6.0, 4.0, 3.0,
    3.0, 2.0, 2.0,
    6.0, 3.0, 3.0,
    10.0, 3.0, 3.0,
    2.0, 2.0, 2.0,
    2.0,
    2.0, 2.0, 2.0,
    2.0, 2.0, 2.0,
    2.0,
    2.0, 2.0, 2.0};
  
  // no actuation at stance
  Vector29 tau_ff_{
    0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0, 
    0.0, 0.0, 0.0,
    0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  
    0.0,
    0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,  
    0.0,
    0.0,  0.0,  0.0};
};
  
/**
 * @brief Current sensor values information of G1 robot
 */
struct G1SensorInfo
{
  /* Position(Angle) values */
  std::vector<double> qIn_;
  /* Velocity values */
  std::vector<double> dqIn_;
  /* Torque values */
  std::vector<double> tauIn_;
  /* Orientation sensor */
  Eigen::Vector3d rpyIn_;
  /* Quaternion */
  Eigen::Quaterniond quatIn_;
  /* Accelerometer */
  Eigen::Vector3d accIn_;
  /* Angular velocity */
  Eigen::Vector3d rateIn_;
};

/**
 * @brief Command data for sending to G1 robot
 */
struct G1CommandData
{
  /* Position(Angle) values */
  std::vector<double> qOut_;
  /* Velocity values */
  std::vector<double> dqOut_;
  /* Torque values */
  std::vector<double> tauOut_;
  /* P gains */
  std::vector<double> kpOut_;
  /* D gains */
  std::vector<double> kdOut_;
};

class G1Control;
template <typename RobotControl, typename RobotSensorInfo, typename RobotCommandData, typename RobotConfigParameter>
class MCControlUnitree2;
  
/**
 * @brief mc_rtc control interface for G1 robot
 */
class G1Control
{
protected:
  typedef enum {
    STATUS_INIT = 0,
    STATUS_WAITING_AIR,
    STATUS_WAITING_GRD,
    STATUS_GAIN_TRANSITION,
    STATUS_RUN,
    STATUS_DAMPING
  } control_status_t;
  
  float control_dt_;
  
  control_status_t status_ = STATUS_INIT;
  control_status_t prev_status_ = STATUS_RUN;

  ControlMode mode_ = ControlMode::Position;
  
  MCControlUnitree2<G1Control, G1SensorInfo, G1CommandData, G1ConfigParameter> * mc_controller_ = nullptr;
  
  mc_rbdyn::Robot* robot_ = nullptr;
  
private:
  /*publisher*/
  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> lowcmd_publisher_;
  /*subscriber*/
  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;
  
  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<BaseState> base_state_buffer_;
  
  /*! Map from joint order to mc_rtc jointId, because mc_rtc jointId is not defined from 0 */
  std::vector<int> refJointOrderToMCJointId_;
  std::unordered_map<int, int> mcJointIdToJointId_;
  
  Vector29 q_init_;
  Vector29 q_lim_lower_;
  Vector29 q_lim_upper_;
  Vector29 q_dot_lim_lower_;
  Vector29 q_dot_lim_upper_;
  Vector29 kp_;  
  Vector29 kd_;
  Vector29 kp_wait_;
  Vector29 kd_wait_;
  Vector29 tau_ff_;
  std::array<float, kNumMotors> tau_des_ = {};

  std::vector<std::string> joint_names_ = {
      "left_hip_yaw_joint",         "left_hip_roll_joint",        "left_hip_pitch_joint", 
      "left_knee_joint",            "left_ankle_pitch_joint",     "left_ankle_roll_joint",    // L-legs        
      "right_hip_yaw_joint",        "right_hip_roll_joint",       "right_hip_pitch_joint",   
      "right_knee_joint",           "right_ankle_pitch_joint",    "right_ankle_roll_joint",   // R-legs
      "waist_yaw_joint",            "waist_roll_joint",           "waist_pitch_joint",        // torso
      "left_shoulder_pitch_joint",  "left_shoulder_roll_joint",   "left_shoulder_yaw_joint", 
      "left_elbow_joint",                                                                     
      "left_wrist_roll_joint",       "left_wrist_pitch_joint",     "left_wrist_yaw_joint",    // L-arms
      "right_shoulder_pitch_joint", "right_shoulder_roll_joint",  "right_shoulder_yaw_joint", 
      "right_elbow_joint",                                                                    
      "right_wrist_roll_joint",       "right_wrist_pitch_joint",     "right_wrist_yaw_joint"};//, // R-arms
      // "NULL"};
  
  float time_ = 0.f;
  float time_run_ = 0.f;
  const float init_duration_ = 5.f;
  const float interp_duration_ = 0.1f;
  
  float report_dt_ = 0.1f;
  
  // multithreading
  unitree::common::ThreadPtr command_writer_ptr_;
#if defined (__ENABLE_RT_PREEMPT__)
  pthread_t control_thread_;
#else
  unitree::common::ThreadPtr control_thread_ptr_;
#endif
  unitree::common::ThreadPtr report_sensor_ptr_;
  
  /* Control loop status */
  bool running_ = false;
  /* Current sensor values information */
  G1SensorInfo stateIn_;
  
  G1CommandData cmdOut_;
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Refresh the quantities in the tables displayed in the console
  ///
  /// \param[in] init Initialize style and header of tables
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  void UpdateTables(bool init = false);
  
  // save <q,dq,tau>i for all joints to the motor state buffer
  void RecordMotorState(const unitree_hg::msg::dds_::LowState_ &msg);
  
  // save imu measurements <gyro,acc,quat,ori> to the base state buffer
  void RecordBaseState(const unitree_hg::msg::dds_::LowState_ &msg);
  
  inline bool IsWeakMotor(int motor_index)
  {
    return motor_index == JointIndex::LeftAnklePitch ||
      motor_index == JointIndex::RightAnklePitch ||
      motor_index == JointIndex::RightShoulderPitch ||
      motor_index == JointIndex::RightShoulderRoll ||
      motor_index == JointIndex::RightShoulderYaw ||
      motor_index == JointIndex::RightElbow ||
      motor_index == JointIndex::LeftShoulderPitch ||
      motor_index == JointIndex::LeftShoulderRoll ||
      motor_index == JointIndex::LeftShoulderYaw ||
      motor_index == JointIndex::LeftElbow;
  }
  
public:
  /**
   * @brief Interface constructor and destructor
   * DDS connection with robot using specified parameters.
   * Control level is set to LOW-level.
   *
   * @param config_param Configuration file parameters
   */
  G1Control(MCControlUnitree2<G1Control, G1SensorInfo, G1CommandData, G1ConfigParameter> * mc_controller, mc_rbdyn::Robot * robot, const G1ConfigParameter & config_param);
  
  virtual ~G1Control();
  
  const G1ConfigParameter & config_;
  
  float time() { return time_; }
  
  float timeRun() { return time_run_; }
  
  const G1SensorInfo & getState() const { return stateIn_; }
  
  const G1CommandData & getCommand() const { return cmdOut_; }
  
  const Vector29 & kp() const { return kp_; }
  
  float kp(int i) { return kp_[i]; }

  void setKp(int i, float kp) { kp_[i] = kp; cmdOut_.kpOut_[i] = kp; }

  const Vector29 & kd() const { return kd_; }
  
  float kd(int i) { return kd_[i]; }

  void setKd(int i, float kd) { kd_[i] = kd; cmdOut_.kdOut_[i] = kd; }
  
  const std::vector<int> & refJointOrderToMCJointId() const { return refJointOrderToMCJointId_; }
  
  int refJointOrderToMCJointId(int i) { return refJointOrderToMCJointId_[i]; }
  
  const std::unordered_map<int, int> & mcJointIdToJointId() const { return mcJointIdToJointId_; }
  
  int mcJointIdToJointId(int i) { return mcJointIdToJointId_[i]; }
  
  void setControlMode(const std::string & mode);
  
  // save <q,dq,kp,kd,tau>i from motor command buffer
  void LowCommandWriter();
  
  // receives lowstate msg and record base and motor state from it
  void LowStateHandler(const void *message);
  
  // control the robot
  void Control();
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Basic print of sensor data to the console
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  void ReportSensors();
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Launch controller once Enter is pressed
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  void endWaiting();
  
  /**
   * @brief Set the initial state values for simulation
   * 
   * @param stance Value defined by RobotModule
   */
  void setInitialState(const std::map<std::string, std::vector<double>> & stance);
  
  /**
   * @brief Loop back the value of "cmdData" to "stateIn"
   * 
   * @param data Command data for sending to G1 robot
   */
  void loopbackState(const G1CommandData & data);
};

} // namespace mc_unitree
