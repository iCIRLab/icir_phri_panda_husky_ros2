#pragma once

//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> 

//ROS Header
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "std_msgs/msg/int16.hpp"
#include "icir_path_planner_ros_interface/srv/plan_mobile_path.hpp"

//SYSTEM Header
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// for hqp controller
#include <icir_hqp_controller/robot/robot_wrapper.hpp>
#include <icir_hqp_controller/formulation/inverse_dynamics_formulation_acc.hpp>
#include <icir_hqp_controller/tasks/task_se3_equality.hpp>
#include <icir_hqp_controller/tasks/task_joint_posture.hpp>
#include <icir_hqp_controller/tasks/task_joint_bound.hpp>
#include <icir_hqp_controller/tasks/task_mobile_base.hpp>
#include <icir_hqp_controller/trajectory/trajectory_euclidian.hpp>
#include <icir_hqp_controller/trajectory/trajectory_se3.hpp>
#include <icir_hqp_controller/solver/solver_HQP_factory.hxx>
// #include <icir_hqp_controller/solver/util.hpp> //multiple definition problem
#include <icir_hqp_controller/math/util.hpp>

using namespace std;
using namespace Eigen;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

enum class JointMoveType { Home, Hold };
enum class WBDMoveType   { Absolute, DeltaInGlobal, DeltaInLocal };

typedef struct State {   
    VectorXd q_;
    VectorXd v_;
    VectorXd dv_;
    VectorXd torque_;
    VectorXd tau_; //use to calculate ddq
} state;  
typedef struct Joint_Action{
    VectorXd kp_, kd_, q_target_;
    int type_;
    double duration_;
    bool is_succeed_{true};

} jointaction;
typedef struct SE3_Action{
    VectorXd kp_, kd_;
    pinocchio::SE3 se3_target_;
    int type_;
    double duration_;
    bool is_wholebody_{true};
    bool is_succeed_{true};

} se3action;

namespace RobotController{
    class FrankaHuskyWrapper{
        public:             
            FrankaHuskyWrapper(const std::string & robot_node, const bool & issimulation, const bool & ismobile, const bool & isrobotiq, const bool & iscalibration, rclcpp::Node::SharedPtr node);
            ~FrankaHuskyWrapper(){};

            void initialize();
            void ctrl_update(const int& ); // msg for chaning controller
            void ctrl_update(const int& msg, int axis, double delta, double duration);
            void ctrl_update(const int& msg, JointMoveType type, double duration);   
            void ctrl_update(const int& msg, WBDMoveType type, const Vector3d& wbd_pos, double duration);
            void initPostureTraj(double duration, JointMoveType type);
            void initMobileTraj(double duration);
            void initEETraj(double duration);
            void updateMobile();
            void updateMobileOri();
            void updatePosture();
            void updateEE();
            void solveHQP();
            void updateAndSolve();
            void franka_update(const sensor_msgs::msg::JointState&); // franka state update //for simulation
            void franka_update(const Vector7d&, const Vector7d&); // franka state update //for experiment            
            void franka_update(const Vector7d& q, const Vector7d& qdot, const Vector7d& tau); // franka state update //for experiment
            void husky_update(const sensor_msgs::msg::JointState&); // husky state update
            void husky_update(const Vector3d&, const Vector3d&, const Vector2d&, const Vector2d&); // husky state update
            void Fext_update(const Vector6d& Fext); //for simulation & experiment

            void compute(const double &); // computation by hqp controller
            void franka_output(VectorXd & qacc); // joint torque of franka 
            void husky_output(VectorXd & qvel); // joint velocity of husky

            void position(pinocchio::SE3 & oMi);
            void position_offset(pinocchio::SE3 & oMi);
            void position_link0_offset(pinocchio::SE3 & oMi);
            void com(Eigen::Vector3d& com);
            void velocity(pinocchio::Motion& vel);
            void velocity_offset(pinocchio::Motion& vel);
            void velocity_origin(pinocchio::Motion& vel);            
            void acceleration(pinocchio::Motion& accel);
            void acceleration_origin(pinocchio::Motion & accel);
            void acceleration_origin2(pinocchio::Motion & accel);
            void force(pinocchio::Force & force);
            void force_origin(pinocchio::Force & force);
            void force_origin2(pinocchio::Force & force);
            void tau(VectorXd & tau_vec);
            void ddq(VectorXd & ddq_vec);

            void mass(MatrixXd & mass_mat);
            void nle(VectorXd & nle_vec);
            void JWorld(MatrixXd & J);
            void JLocal(MatrixXd & Jo);
            void JLocal_offset(MatrixXd & Jo);
            void dJLocal(MatrixXd & dJo);
            void dJLocal_offset(MatrixXd & dJo);
            void g(VectorXd & g_vec);
            void g_joint7(VectorXd & g_vec);
            void g_local_offset(VectorXd & g_vec);
            void Fh_gain_matrix(MatrixXd & Fh, MatrixXd & Fh_local, bool isinteractiongain);
            void MxLocal_offset(MatrixXd & Mx);
            void MxGlobal(MatrixXd & Mx);

            void ee_state(Vector3d & pos, Eigen::Quaterniond & quat);
            void base_state(Vector3d & base);

            void rotx(double & a, Eigen::Matrix3d & rot);
            void roty(double & a, Eigen::Matrix3d & rot);
            void rotz(double & a, Eigen::Matrix3d & rot);

            MatrixXd skew_matrix(const VectorXd& vec);
            pinocchio::SE3 vel_to_SE3(VectorXd vel, double dt); 
            void get_dt(double dt);
            double trajectory_length_in_time();
            double noise_elimination(double x, double limit);
            void eeoffset_update();
            void R_joint7();
            void EstimationMotion();
            void resetAllTasks();
            void setMujocoObstacles(); // fill path-planner request from the /<ns>/obstacles rosparam (auto-loaded from mujoco obs_* geoms)

            int ctrltype(){
                return ctrl_mode_;
            }
            
            void state(State & state_robot){
                state_robot = state_;
            }

        private:
            // 파라미터를 (최초 1회) 선언하고 값을 읽는다. launch가 넘긴 값이 있으면
            // 그 값이, 없으면 fallback이 나온다. ROS1 getParam/param 자리를 대신한다.
            template <typename T>
            T declare_or_get(const std::string & name, const T & fallback){
                if (!node_->has_parameter(name)) node_->declare_parameter<T>(name, fallback);
                return node_->get_parameter(name).get_value<T>();
            }

            bool issimulation_, mode_change_, fine_adjust_init_, trjectory_end_, mobile_path_succeed_;
            double stime_, time_, node_index_, node_num_, prev_node_;
            std::string robot_node_;
            State state_;
            Joint_Action joint_action_;
            SE3_Action se3_action_;                  


            int ctrl_mode_;
            Eigen::VectorXd q_ref_;
            pinocchio::SE3 H_ee_ref_, H_mobile_ref_, T_offset_, T_vel_;
            Vector3d ee_offset_, obj_length_global_, obj_length_local_;
            MatrixXd Adj_mat_, hGr_local_, hGr_local_Null_, hGr_local_pinv_, R_joint7_atHome_, R_joint7_atTask_;
            double est_time_, dt_;
            double joint7_to_finger_;
            Vector6d Fext_, Fext_calibration_, Dinv_gain_;
            bool initial_calibration_update_, getcalibration_;     
            MatrixXd Me_inv_, P_global_, P_local_, Fh_, Fh_local_;       
            ofstream fout_;  
            double traj_length_in_time_, target_distance_, target_time_, target_velocity_;   
            bool ismobile_, isrobotiq_;
            std::vector<pinocchio::SE3> precomputed_mobile_path_;
            MatrixXd Me_;
            int    ee_jog_axis_;    // 0:x, 1:y, 2:z
            double ee_jog_delta_,ee_jog_duration_;  

            VectorXd q_goal_;
            double   joint_duration_;
            JointMoveType joint_move_type_;

            Vector3d wbd_pos_;
            double   wbd_duration_;
            WBDMoveType wbd_move_type_;

            //hqp
            std::shared_ptr<icirhqp::robot::RobotWrapper> robot_;
            pinocchio::Model model_;
            pinocchio::Data data_;

            std::shared_ptr<icirhqp::InverseDynamicsFormulationAccForce> tsid_;
            
            std::shared_ptr<icirhqp::tasks::TaskJointPosture> postureTask_;
            std::shared_ptr<icirhqp::tasks::TaskSE3Equality> eeTask_;
            std::shared_ptr<icirhqp::tasks::TaskJointBounds> torqueBoundsTask_;            
            std::shared_ptr<icirhqp::tasks::TaskMobileEquality> mobileTask_ori_, mobileTask_trans_;

            std::shared_ptr<icirhqp::trajectory::TrajectoryEuclidianCubic> trajPosture_Cubic_;
            std::shared_ptr<icirhqp::trajectory::TrajectoryEuclidianConstant> trajPosture_Constant_;
            std::shared_ptr<icirhqp::trajectory::TrajectoryEuclidianTimeopt> trajPosture_Timeopt_;
            std::shared_ptr<icirhqp::trajectory::TrajectorySE3Cubic> trajEE_Cubic_, trajMobile_Cubic_;
            std::shared_ptr<icirhqp::trajectory::TrajectorySE3Constant> trajEE_Constant_, trajMobile_Constant_;
            std::shared_ptr<icirhqp::trajectory::TrajectorySE3Timeopt> trajEE_Timeopt_, trajMobile_Timeopt_;            

            icirhqp::trajectory::TrajectorySample sampleMobile_, sampleEE_, samplePosture_;

            icirhqp::solver::SolverHQPBase * solver_;
            
            int na_, nq_, nv_;
            int cnt_;

            //service
            //
            // ROS1은 mobile_path_client_.call(srv)로 제어 루프 안에서 동기 호출했다.
            // rclcpp에서 스핀 중인 노드가 자기 실행자 위에서 동기 호출하면 데드락이므로,
            // 서비스 전용 노드를 따로 두고 그 노드만 spin_until_future_complete로 돌린다.
            // 호출 지점의 "요청 채우고 -> 기다렸다가 -> 응답 쓴다" 흐름은 ROS1 그대로다.
            using PlanMobilePath = icir_path_planner_ros_interface::srv::PlanMobilePath;
            rclcpp::Node::SharedPtr srv_node_;
            rclcpp::Client<PlanMobilePath>::SharedPtr mobile_path_client_;
            PlanMobilePath::Request  mobile_path_req_;
            PlanMobilePath::Response mobile_path_res_;

            // ROS1의 mobile_path_client_.call() 대체. 응답을 mobile_path_res_에 채우고
            // 성공 여부를 돌려준다.
            bool callMobilePathPlanner();

            //ros
            rclcpp::Node::SharedPtr node_;
    };
} // namespace


