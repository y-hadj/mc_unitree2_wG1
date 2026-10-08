#pragma once

#include <chrono>
#include <thread>
#include <Eigen/Core>
#include <vector>
#include <map>

#include <fort.hpp>

//#define __ENABLE_RT_PREEMPT__

#if defined(__ENABLE_RT_PREEMPT__)
#include <pthread.h>
#include "rtapi.h"
#endif

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>

// located in unitree_sdk2/example/go2/low_level
#include <go2/low_level/base_state.h>
#include <go2/low_level/data_buffer.hpp>
#include <go2/low_level/motors.hpp>

#include <mc_control/mc_controller.h>
#include "MCControlUnitree2.h"

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

using Vector12 = Eigen::Matrix<float, 12, 1>;

namespace mc_unitree
{
  constexpr double PosStopF = (2.146E+9f);
  constexpr double VelStopF = (16000.0f);
  
  const std::string ROBOT_NAME = "go2";
  const std::string CONFIGURATION_FILE = "/usr/local/etc/mc_unitree/mc_rtc.yaml";
  
  const int jointIdsToMotorIds[12] =
    {JointIndex::FL_hip_joint,
     JointIndex::FL_thigh_joint,
     JointIndex::FL_calf_joint,
     JointIndex::FR_hip_joint,
     JointIndex::FR_thigh_joint,
     JointIndex::FR_calf_joint,
     JointIndex::RL_hip_joint,
     JointIndex::RL_thigh_joint,
     JointIndex::RL_calf_joint,
     JointIndex::RR_hip_joint,
     JointIndex::RR_thigh_joint,
     JointIndex::RR_calf_joint
    };
  
/**
 * @brief Configuration file parameters for mc_unitree
 */
struct Go2ConfigParameter
{
  Go2ConfigParameter()
  : network_(""), mode_(ControlMode::Position)
  {}
  /* Communication information with a real robot */
  /* Connection network */
  std::string network_;
  /* ControlMode : Position/Velocity/Torque (Velocity and Torque are not supported)*/
  ControlMode mode_ = ControlMode::Position;
  
  // Default configuration
  // This will be overwritten by stance defined in mc_h1
  Vector12 q_init_{
    0.0, 0.67, -1.3, // FL
    0.0, 0.67, -1.3, // FR
    0.0, 0.67, -1.3, // RL
    0.4, 0.67, -1.3  // RR
  };
  Vector12 q_lim_lower_{
    -1.0472, -1.5708, -2.7227,
    -1.0472, -1.5708, -2.7227,
    -1.0472, -1.5708, -2.7227,
    -1.0472, -1.5708, -2.7227
  };
  Vector12 q_lim_upper_{
    1.0472, 3.4907, -0.83776,
    1.0472, 3.4907, -0.83776,
    1.0472, 3.4907, -0.83776,
    1.0472, 3.4907, -0.83776,
  };
  
  // Proportional derivative gains
  Vector12 kp_{
    5.0, 5.0, 5.0,
    5.0, 5.0, 5.0,
    5.0, 5.0, 5.0,
    5.0, 5.0, 5.0
  };
  
  Vector12 kd_{
    1.0, 1.0, 1.0,
    1.0, 1.0, 1.0,
    1.0, 1.0, 1.0,
    1.0, 1.0, 1.0
  }; 
  
  Vector12 kp_stand_{
    50.0, 50.0, 50.0,
    50.0, 50.0, 50.0,
    50.0, 50.0, 50.0,
    50.0, 50.0, 50.0
  };
  
  Vector12 kd_stand_{
    5.0, 5.0, 5.0,
    5.0, 5.0, 5.0,
    5.0, 5.0, 5.0,
    5.0, 5.0, 5.0
  };
  
  Vector12 tau_ff_{
    0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,
    0.0,  0.0,  0.0,
    0.0,  0.0,  0.0
  };
};

/**
 * @brief Current sensor values information of Go2 robot
 */
struct Go2SensorInfo
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
  /* Foot force sensors */
  std::vector<double> footForceIn_;
};

/**
 * @brief Command data for sending to Go2 robot
 */
struct Go2CommandData
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

class Go2Control;
template <typename RobotControl, typename RobotSensorInfo, typename RobotCommandData, typename RobotConfigParameter>
class MCControlUnitree2;
  
/**
 * @brief mc_rtc control interface for Go2 robot
 */
class Go2Control
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
  
  MCControlUnitree2<Go2Control, Go2SensorInfo, Go2CommandData, Go2ConfigParameter> * mc_controller_ = nullptr;
  
  mc_rbdyn::Robot* robot_ = nullptr;
  
private:
  const Go2ConfigParameter & config_;
  
  /*publisher*/
  unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher_;
  /*subscriber*/
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber_;
  
  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<BaseState> base_state_buffer_;
  
  Vector12 q_init_;
  Vector12 q_lim_lower_;
  Vector12 q_lim_upper_;
  Vector12 kp_;  
  Vector12 kd_;
  Vector12 kp_wait_;
  Vector12 kd_wait_;
  Vector12 tau_ff_;
  std::array<float, kNumMotors> tau_des_ = {};
  
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
  
  // Table for console display
  fort::char_table table_IMU_;
  fort::char_table table_legs_;
  fort::char_table table_misc_;
  fort::char_table table_legs_cmd_;
  
  /* Control loop status */
  bool running_ = false;
  /* Current sensor values information */
  Go2SensorInfo stateIn_;
  
  Go2CommandData cmdOut_;
  
  ////////////////////////////////////////////////////////////////////////////////////////////////
  ///
  /// \brief Refresh the quantities in the tables displayed in the console
  ///
  /// \param[in] init Initialize style and header of tables
  ///
  ////////////////////////////////////////////////////////////////////////////////////////////////
  void UpdateTables(bool init = false);
  
  void RecordMotorState(const unitree_go::msg::dds_::LowState_ &msg);
  
  void RecordBaseState(const unitree_go::msg::dds_::LowState_ &msg);
  
public:
  /**
   * @brief Interface constructor and destructor
   * DDS connection with robot using specified parameters.
   * Control level is set to LOW-level.
   *
   * @param config_param Configuration file parameters
   */
  Go2Control(MCControlUnitree2<Go2Control, Go2SensorInfo, Go2CommandData, Go2ConfigParameter> * mc_controller, mc_rbdyn::Robot * robot, const Go2ConfigParameter & config_param);
  
  virtual ~Go2Control();
      
  float time() { return time_; }
  
  float timeRun() { return time_run_; }
  
  const Go2SensorInfo & getState() const { return stateIn_; }
  
  const Go2CommandData & getCommand() const { return cmdOut_; }
  
  const Vector12 & kp() const { return kp_; }
  
  float kp(int i) { return kp_[i]; }
  
  void setKp(int i, float kp) { kp_[i] = kp; }
  
  const Vector12 & kd() const { return kd_; }
  
  float kd(int i) { return kd_[i]; }
  
  void setKd(int i, float kd) { kd_[i] = kd; }
  
  int refJointOrderToMCJointId(int i) { return i; }
  
  int mcJointIdToJointId(int i) { return i; }
  
  void LowCommandWriter();
  
  void LowStateHandler(const void *message);
  
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
   * @param data Command data for sending to Go2 robot
   */
  void loopbackState(const Go2CommandData & data);
};

} // namespace mc_unitree
