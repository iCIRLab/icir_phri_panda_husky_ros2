// phri_simul (ROS1) 의 ROS2 포팅본.
//
// 구조는 ROS1 그대로다: 독립 실행 노드가 1kHz 루프를 직접 돌리고, MuJoCo 브릿지와는
// 토픽으로만 이야기한다 (컨트롤러를 ros2_control 플러그인으로 만들지 않는다).
// 브릿지가 커스텀 mujoco_ros 에서 mujoco_ros2_control 의 MujocoRosInterfacePlugin 으로
// 바뀌었을 뿐, 토픽 이름과 JointSet 인덱스 규약은 ROS1과 같다.
#pragma once

// Controller
#include "icir_phri_panda_husky/panda_husky_hqp.h"

//Mujoco MSG Header
#include "mujoco_ros2_control_msgs/msg/joint_set.hpp"

// Ros
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "icir_phri_msgs/msg/object_parameter.hpp"

// System — keyboard_fd() 가 /dev/tty 를 여는 데 쓴다
#include <fcntl.h>

// Tf
#include <tf2_ros/transform_broadcaster.h>

// Object estimation
// ROS1은 상대경로로 옆 패키지 소스를 직접 찔렀다. 이제 정상 ament 패키지라 include
// 경로로 받는다.
#include <icir_object_estimation/main/extendedkalman.hpp>
#include <icir_object_estimation/objdyn/object_dynamics.hpp>

rclcpp::Node::SharedPtr node_;

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mujoco_command_pub_;
rclcpp::Publisher<mujoco_ros2_control_msgs::msg::JointSet>::SharedPtr robot_command_pub_;
rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mujoco_run_pub_;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_mesured_pub_;
rclcpp::Publisher<icir_phri_msgs::msg::ObjectParameter>::SharedPtr object_parameter_pub_;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_, accel_pub_;
rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr ee_state_pub_;
rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr base_state_pub_;

mujoco_ros2_control_msgs::msg::JointSet robot_command_msg_;
geometry_msgs::msg::Transform ee_state_msg_;
sensor_msgs::msg::JointState base_state_msg_;

double mujoco_time_, time_, dt, est_time_;
bool isgrasp_, ismobile_, issimulation_, isrobotiq_, iscalibration_;
Eigen::VectorXd franka_qacc_, husky_qacc_, robot_nle_, robot_g_, franka_torque_, Fext_cali_;
Eigen::MatrixXd robot_mass_, robot_J_local_, robot_dJ_local_, robot_J_world_;
string group_name;
std_msgs::msg::Bool sim_run_msg_;

RobotController::FrankaHuskyWrapper * ctrl_;
State state_;
std::shared_ptr<tf2_ros::TransformBroadcaster> br_;

void simCommandCallback(const std_msgs::msg::String::SharedPtr msg);
void simTimeCallback(const std_msgs::msg::Float32::SharedPtr msg);
void JointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
void ctrltypeCallback(const std_msgs::msg::Int16::SharedPtr msg);
void joint_states_publish(const sensor_msgs::msg::JointState& msg);

void setRobotCommand();
void setGripperCommand();
void getEEState();
void getBaseState();

//////////////////// object estimation /////////////////
EKF * ekf;
Objdyn objdyn;

bool isstartestimation_, isobjectdynamics_, isFextapplication_, isFextcalibration_;
double n_param, m_FT, traj_length_in_time_;
Eigen::MatrixXd A, H, Q, R, P;
Eigen::VectorXd h, FT_measured, param, robot_g_local_, FT_object_, param_true_, Fext_global_;
pinocchio::Motion vel_param, acc_param;
VectorXd ddq_mujoco, tau_estimated, tau_ext, v_mujoco, a_mujoco, a_mujoco_filtered;

void getObjParam_init();
void getObjParam();
void ObjectParameter_pub();
double saturation(double x, double limit);
void FT_measured_pub();
void vel_accel_pub();
////////////////////////////////////////////////////////

void EEStatusCallback(geometry_msgs::msg::Transform& msg, const pinocchio::SE3 & H_ee);

void process_key(int key);
void keyboard_event();
void remoteKeyCallback(const std_msgs::msg::String::SharedPtr msg);

// 키를 읽을 파일 디스크립터.
//
// ROS1 roslaunch 는 자식 노드에 부모 터미널의 stdin 을 그대로 물려줬고, `-keyboard`
// 경로가 fd 0 을 그대로 읽으면 됐다. ros2 launch 는 자식 stdin 을 /dev/null 로 막기
// 때문에 fd 0 으로는 아무것도 못 읽는다. fd 0 이 터미널이 아니면 제어 터미널을
// 직접 연다 — 런치 터미널에 그대로 키를 칠 수 있다.
inline int keyboard_fd()
{
    static int fd = -2;  // -2: 아직 안 봄, -1: 터미널 없음
    if (fd == -2) {
        if (isatty(0)) {
            fd = 0;
        } else {
            fd = open("/dev/tty", O_RDONLY | O_NONBLOCK);  // nohup 등이면 실패한다
        }
    }
    return fd;
}

bool _kbhit()
{
    const int fd = keyboard_fd();
    if (fd < 0) return false;

    termios term;
    if (tcgetattr(fd, &term) != 0) return false;

    termios term2 = term;
    term2.c_lflag &= ~ICANON;
    tcsetattr(fd, TCSANOW, &term2);

    int byteswaiting = 0;
    ioctl(fd, FIONREAD, &byteswaiting);

    tcsetattr(fd, TCSANOW, &term);

    return byteswaiting > 0;
};
