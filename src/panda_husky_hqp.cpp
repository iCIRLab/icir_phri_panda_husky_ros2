#include "icir_phri_panda_husky/panda_husky_hqp.h"
#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace pinocchio;
using namespace Eigen;
using namespace std;
using namespace icirhqp;
using namespace icirhqp::trajectory;
using namespace icirhqp::math;
using namespace icirhqp::tasks;
using namespace icirhqp::solver;
using namespace icirhqp::robot;
using namespace icirhqp::contacts;

namespace {
    // planar yaw from a 2D mobile SE3 (matches getMobilePosition's rotation convention)
    inline double mobileYaw(const pinocchio::SE3 & H){
        return atan2(-H.rotation()(0,1), H.rotation()(0,0));
    }
    // write a planar pose (x, y, yaw) into an SE3 (z / roll / pitch left untouched)
    inline void setPlanarPose(pinocchio::SE3 & H, double x, double y, double theta){
        H.translation()(0) = x;
        H.translation()(1) = y;
        H.rotation().setIdentity();
        H.rotation()(0,0) =  cos(theta);
        H.rotation()(0,1) = -sin(theta);
        H.rotation()(1,0) =  sin(theta);
        H.rotation()(1,1) =  cos(theta);
    }
}

namespace RobotController{
    FrankaHuskyWrapper::FrankaHuskyWrapper(const std::string & robot_node, const bool & issimulation, const bool & ismobile, const bool & isrobotiq, const bool & iscalibration, rclcpp::Node::SharedPtr node)
    : robot_node_(robot_node), issimulation_(issimulation), ismobile_(ismobile), isrobotiq_(isrobotiq), getcalibration_(iscalibration), node_(node)
    {
        time_ = 0.;        
        node_index_ = 0;
        cnt_ = 0;

        mode_change_ = false;
        ctrl_mode_ = 0;                
    }

    void FrankaHuskyWrapper::initialize(){        
        // Robot for pinocchio
        // ROS1은 "/<ns>/robot_urdf_path" 처럼 전역 파라미터 서버를 읽었다. ROS2엔
        // 전역 서버가 없어 launch가 이 노드의 파라미터로 직접 넘긴다 -> 이름에서
        // 네임스페이스가 빠진다. 값과 의미는 그대로다.
        const string model_path = declare_or_get<string>("robot_urdf_path", "");
        const string urdf_name  = declare_or_get<string>("robot_urdf", "");  //"panda_arm_hand_l.urdf" w/o mobile, "husky_panda_hand.urdf" w/ mobile
        if (model_path.empty() || urdf_name.empty())
            throw std::runtime_error("robot_urdf_path / robot_urdf parameter is empty");

        vector<string> package_dirs;
        package_dirs.push_back(model_path);
        string urdfFileName = package_dirs[0] + urdf_name;

        robot_ = std::make_shared<RobotWrapper>(urdfFileName, package_dirs, ismobile_, false); //first false : w/o mobile, true : w/ mobile
        model_ = robot_->model();
        
        //nq_/nv_/na_ is # of joint w.r.t pinocchio model ("panda_arm_hand_l.urdf"), so there is no gripper joints
        nq_ = robot_->nq(); //12 : odom (3 - x,y,theta) + husky (2) + franka (7)
        nv_ = robot_->nv(); //12
        na_ = robot_->na(); //9  : nv-3, odom dof eliminated

        // State (for pinocchio)
        state_.q_.setZero(nq_);
        state_.v_.setZero(nv_);
        state_.dv_.setZero(nv_);
        state_.torque_.setZero(na_);
        state_.tau_.setZero(na_);

        // tsid
        tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);
        tsid_->computeProblemData(time_, state_.q_, state_.v_);
        data_ = tsid_->data();

        // tasks
        postureTask_ = std::make_shared<TaskJointPosture>("task-posture", *robot_);
        
        VectorXd posture_gain;
        if (ismobile_) posture_gain.resize(na_-2);
        else           posture_gain.resize(na_);
        
        if (!issimulation_) //for real
            posture_gain << 100., 100., 100., 200., 200., 200., 200.;
        else // for simulation        	
            // posture_gain << 40000., 40000., 40000., 40000., 40000., 40000., 40000.;
        	posture_gain << 4000., 4000., 4000., 4000., 4000., 4000., 4000.;

        postureTask_->Kp(posture_gain);
        postureTask_->Kd(2.0*postureTask_->Kp().cwiseSqrt());
        
        //////////////////// EE offset ////////////////////////////////////////        
        //When offset is applied, the control characteristics seems to be changed (gain tuning is needed)
        //This offset is applied to both reference (in this code) and feedback (in task_se3_equality.cpp)        
        if (isrobotiq_) joint7_to_finger_ = 0.247; //0.247 (z-axis) = 0.222(distance from link7 to left_inner_finger) + 0.025(finger center length)         
        else            joint7_to_finger_ = 0.2054; //0.2054 (z-axis) = 0.107(distance from link7 to EE) + 0.0584(hand length) + 0.04(finger length)          
        this->eeoffset_update(); 
        
        this->R_joint7();   //make rotation matrix of joint7 from global to local          

        //////////////////// human grasp position (object length) //////////////////////////////////////// 
        //define object length 
        obj_length_global_ = Vector3d(0.0, -1.5, 0.0); // 1.5 length in global y axis                                
        obj_length_local_  = R_joint7_atHome_ * obj_length_global_; // global to local                

        this->EstimationMotion();
        
        //trajectory length in time for object parameter estimation
        traj_length_in_time_ = 10.0;    
        ///////////////////////////////////////////////////////////////////////        

        VectorXd ee_gain(6);        
        if (!issimulation_) { //for real                
            ee_gain << 100., 100., 100., 400., 400., 600.;                
        }
        else { //for simulation
            ee_gain << 1000., 1000., 1000., 2000., 2000., 2000.;
        }                                
        
        eeTask_ = std::make_shared<TaskSE3Equality>("task-se3", *robot_, "panda_joint7", ee_offset_); //here, ee_offset_ is applied to current pos value
        // 원본은 `ee_gain * Vector::Ones(6)` 였다. 6x1 * 6x1 은 잘못된 행렬곱이라
        // Eigen 이 assert 를 건다. ROS1은 Release(NDEBUG) 빌드라 assert 가 꺼져 있었고,
        // 곱의 inner dimension 이 lhs.cols()=1 이라 결과가 ee_gain(i)*1.0, 즉 ee_gain
        // 그 자체였다. 값이 같으므로 의도대로 바로 넘긴다.
        eeTask_->Kp(ee_gain);
        // eeTask_->Kd(2.0*eeTask_->Kp().cwiseSqrt());        
        eeTask_->Kd(1.0*eeTask_->Kp().cwiseSqrt());                        

        torqueBoundsTask_ = std::make_shared<TaskJointBounds>("task-torque-bounds", *robot_);
        icirhqp::math::Vector dq_max = 500000.0*icirhqp::math::Vector::Ones(na_);
        dq_max(0) = 500.; //? 
        dq_max(1) = 500.; //?
        icirhqp::math::Vector dq_min = -dq_max;
        torqueBoundsTask_->setJointBounds(dq_min, dq_max);        

        if (ismobile_) {
            mobileTask_ori_ = std::make_shared<TaskMobileEquality>("task-mobile-ori", *robot_, true); //ori
            mobileTask_ori_->Kp(400.0*Vector3d::Ones());
            mobileTask_ori_->Kd(2.0*mobileTask_ori_->Kp().cwiseSqrt());

            mobileTask_trans_ = std::make_shared<TaskMobileEquality>("task-mobile-trans", *robot_, false); //trans
            mobileTask_trans_->Kp(400.0*Vector3d::Ones());
            mobileTask_trans_->Kd(2.0*mobileTask_trans_->Kp().cwiseSqrt());
        }

        // trajecotries
        sampleEE_.resize(12, 6); //12=3(translation)+9(rotation matrix), 6=3(translation)+3(rotation)

        if (ismobile_) samplePosture_.resize(na_-2); //na_=9 (husky 2 + franka 7)
        else           samplePosture_.resize(na_); //na_=7 franka 7

        trajPosture_Cubic_ = std::make_shared<TrajectoryEuclidianCubic>("traj_posture");
        trajPosture_Constant_ = std::make_shared<TrajectoryEuclidianConstant>("traj_posture_constant");
        trajPosture_Timeopt_ = std::make_shared<TrajectoryEuclidianTimeopt>("traj_posture_timeopt");

        trajEE_Cubic_ = std::make_shared<TrajectorySE3Cubic>("traj_ee");
        trajEE_Constant_ = std::make_shared<TrajectorySE3Constant>("traj_ee_constant");
        Vector3d Maxvel_ee = Vector3d::Ones()*0.2;
        Vector3d Maxacc_ee = Vector3d::Ones()*0.2;
        trajEE_Timeopt_ = std::make_shared<TrajectorySE3Timeopt>("traj_ee_timeopt", Maxvel_ee, Maxacc_ee);
        
        if (ismobile_) {
            sampleMobile_.resize(12, 6);
            trajMobile_Cubic_ = std::make_shared<TrajectorySE3Cubic>("traj_mobile");
            trajMobile_Constant_ = std::make_shared<TrajectorySE3Constant>("traj_mobile_constant");
            Vector3d Maxvel_base = Vector3d::Ones()*1.0;
            Vector3d Maxacc_base = Vector3d::Ones()*1.0;
            trajMobile_Timeopt_ = std::make_shared<TrajectorySE3Timeopt>("traj_mobile_timeopt", Maxvel_base, Maxacc_base);
        }

        // solver
        solver_ = SolverHQPFactory::createNewSolver(SOLVER_HQP_QPOASES, "qpoases");

        // service
        // 서버(MobilePathPlannerServer)가 상대 이름으로 서비스를 열기 때문에 이름은
        // 네임스페이스 아래의 "plan_mobile_path" 다 (ROS1의 노드 private 이름과 다름).
        // 전용 노드를 컨트롤러와 같은 네임스페이스에 만들어 붙인다.
        srv_node_ = std::make_shared<rclcpp::Node>(
            "phri_mobile_path_client", node_->get_namespace());
        mobile_path_client_ = srv_node_->create_client<PlanMobilePath>("plan_mobile_path");
        initial_calibration_update_ = false;

        //ctrol parameters
        ee_jog_axis_ = 0; // 0:x, 1:y, 2:z
        ee_jog_delta_ = 0.0;
        ee_jog_duration_ = 2.0; 
        joint_move_type_ = JointMoveType::Home;
        joint_duration_  = 3.0;  
        wbd_pos_ = Vector3d::Zero();
        wbd_duration_ = 2.0;
        wbd_move_type_ = WBDMoveType::DeltaInGlobal;     

        //initial posture
        q_ref_.setZero(7);
        q_ref_(0) = 0.0;//M_PI /4.0;
        q_ref_(1) = 0.0 * M_PI / 180.0;
        q_ref_(3) = -M_PI / 2.0;
        q_ref_(5) = M_PI/ 2.0;        
        
        if (isrobotiq_) q_ref_(6) = -M_PI/ 2.0;                
        else            q_ref_(6) = -M_PI/ 4.0;        

        //inertia shaping --------------------------------//
        Me_.resize(6,6);
        Me_inv_.resize(6,6);
        Me_inv_.setIdentity();                 
        eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
        if (ismobile_) {
            mobileTask_ori_->setDesiredinertia(MatrixXd::Identity(6,6));
            mobileTask_trans_->setDesiredinertia(MatrixXd::Identity(6,6));
        }
        //-----------------------------------------------//

        //Fh gain in global ------------------------------//        
        Fh_.resize(6,6);    
        Fh_.setZero();                 
        
        //case.1 :use translational axis        
        // Fh_.topLeftCorner(3,3).setIdentity();                
        
        //case.2 : only use rx-axis to move z-axis 
        Fh_(2,3) = -1.0 * 4.0;     
        //-----------------------------------------------//
        
        //Fh gain in local ------------------------------//
        Fh_local_.resize(6,6) ;        
        Fh_local_.setZero();
        
        //case.1 : use local rz-axis to move local y-axis
        Fh_local_(1,5) = -5.0;  
        //-----------------------------------------------//

        cout << " " << endl;
        cout << "Fh_" << endl;
        cout << Fh_ << endl;
        cout << " " << endl;
        cout << "Fh_local_" << endl;
        cout << Fh_local_ << endl;    
    }      

    void FrankaHuskyWrapper::EstimationMotion(){ 
        //define hGr matrix in local coordinate
        hGr_local_.resize(3,6);
        hGr_local_.topLeftCorner(3,3).setIdentity();         
        hGr_local_.topRightCorner(3,3) = -1 * skew_matrix(obj_length_local_);

        cout << " " << endl;
        cout << "hGr_local" << endl;
        cout << hGr_local_ << endl;
        
        hGr_local_pinv_.resize(6,3);
        hGr_local_pinv_ = hGr_local_.completeOrthogonalDecomposition().pseudoInverse();

        cout << " " << endl;
        cout << "hGr_local_pinv_" << endl;
        cout << hGr_local_pinv_ << endl;

        hGr_local_Null_.resize(6,6);
        hGr_local_Null_ = MatrixXd::Identity(6,6) - hGr_local_pinv_* hGr_local_;

        cout << " " << endl;
        cout << "hGr_local_Null_" << endl;
        cout << hGr_local_Null_ << endl;

        //define P matrix
        P_global_.resize(6,6);
        P_global_.setIdentity();
        P_global_.bottomLeftCorner(3,3) = -1*skew_matrix(obj_length_global_);        

        P_local_.resize(6,6);
        P_local_.setIdentity();
        P_local_.bottomLeftCorner(3,3) = -1*skew_matrix(obj_length_local_);
    }
    
    void FrankaHuskyWrapper::R_joint7(){ 
        Matrix3d Rx, Ry, Rz;
        double anglex, angley, anglez;         
        if (isrobotiq_) {
            anglex = -180.0*M_PI/180.0;
            anglez =   90.0*M_PI/180.0;
            this->rotx(anglex, Rx);
            this->rotz(anglez, Rz);   
            R_joint7_atHome_ = Rz * Rx;            
        }
        else { //franka gripper
            anglex = -180.0*M_PI/180.0;
            anglez =   45.0*M_PI/180.0;
            this->rotx(anglex, Rx);
            this->rotz(anglez, Rz);   
            R_joint7_atHome_ = Rz * Rx;            
        }       
        cout << " " << endl;
        cout << "R_joint7_atHome_" << endl;
        cout << R_joint7_atHome_ << endl;            
    }

    void FrankaHuskyWrapper::eeoffset_update(){ 
        ee_offset_ = Vector3d(0.0, 0.0, joint7_to_finger_); //w.r.t joint7

        T_offset_.setIdentity();
        T_offset_.translation(ee_offset_);                

        Adj_mat_.resize(6,6);
        Adj_mat_.setIdentity();
        Adj_mat_.topRightCorner(3,3) = -1 * skew_matrix(ee_offset_); //due to "A cross B = -B cross A"
    }

    void FrankaHuskyWrapper::franka_update(const sensor_msgs::msg::JointState& msg){ //for simulation (mujoco)
        // mujoco callback msg
        // msg.position : 7(odom) + 4(wheel) + 7(joint) + 2(gripper)
        // msg.velocity : 6(odom) + 4(wheel) + 7(joint) + 2(gripper) 

        assert(issimulation_);        
        for (int i=0; i< 7; i++){
            if (ismobile_) {
                state_.q_(i+5) = msg.position[i+11];
                state_.v_(i+5) = msg.velocity[i+10];            
            }
            else {                
                state_.q_(i) = msg.position[i];
                state_.v_(i) = msg.velocity[i];
            }
        }        
    }
    void FrankaHuskyWrapper::franka_update(const Vector7d& q, const Vector7d& qdot){ //for experiment
        assert(!issimulation_);
        state_.q_.tail(7) = q;
        state_.v_.tail(7) = qdot;
    }        

    void FrankaHuskyWrapper::franka_update(const Vector7d& q, const Vector7d& qdot, const Vector7d& tau){ //for experiment, use pinocchio::aba
        assert(!issimulation_);
        state_.q_.tail(7) = q;
        state_.v_.tail(7) = qdot;

        state_.tau_.tail(7) = tau;
    }     

    void FrankaHuskyWrapper::husky_update(const sensor_msgs::msg::JointState& msg){ //for simulation (mujoco)
        // mujoco callback msg
        // msg.position : 7(odom) + 4(wheel) + 7(joint) + 2(gripper)
        // msg.velocity : 6(odom) + 4(wheel) + 7(joint) + 2(gripper)

        assert(issimulation_);
        for (int i=0; i<2; i++){ //odom x, y
            state_.q_(i) = msg.position[i];
            state_.v_(i) = msg.velocity[i];
        }

        double theta = atan2(2.* (msg.position[5] * msg.position[4] + msg.position[6] * msg.position[3]), 1- 2.*(pow( msg.position[6], 2) + pow(msg.position[5], 2))); //odom yaw
        state_.q_(2) = theta;
        state_.v_(2) = msg.velocity[5];

        // only for front wheel (not used)
        state_.q_(3) = msg.position[7];
        state_.q_(4) = msg.position[8];
        state_.v_(3) = msg.velocity[6];
        state_.v_(4) = msg.velocity[7];
    }

    void FrankaHuskyWrapper::husky_update(const Vector3d& base_pos, const Vector3d& base_vel, const Vector2d& wheel_pos, const Vector2d& wheel_vel){ //for experiment
        assert(!issimulation_);
        for (int i=0; i<3; i++){
            state_.q_(i) = base_pos(i);
            state_.v_(i) = base_vel(i);
        }
        for (int i=0; i<2; i++){
            state_.q_(i+3) = wheel_pos(i);
            state_.v_(i+3) = wheel_vel(i);
        }
    }

    void FrankaHuskyWrapper::Fext_update(const Vector6d& Fext){ //for simulation & experiment
        Fext_ = Fext;
    }   

    void FrankaHuskyWrapper::ctrl_update(const int& msg){
        ctrl_mode_ = msg;
        mode_change_ = true;
        RCLCPP_INFO(node_->get_logger(), "[ctrltypeCallback] %d", ctrl_mode_);
    }

    //ctrl_update for EE_jog
    void FrankaHuskyWrapper::ctrl_update(const int& msg, int axis, double delta, double duration){
        ee_jog_axis_     = axis;
        ee_jog_delta_    = delta;
        ee_jog_duration_ = duration;
        ctrl_mode_    = msg;
        mode_change_  = true;
        RCLCPP_INFO(node_->get_logger(), "[ctrltypeCallback] mode=%d, axis=%d, delta=%.3f, dur=%.2f", ctrl_mode_, ee_jog_axis_, ee_jog_delta_, ee_jog_duration_);
    }

    //ctrl_update for joint posture
    void FrankaHuskyWrapper::ctrl_update(const int& msg, JointMoveType type, double duration){
        joint_move_type_ = type;
        joint_duration_  = duration;
        ctrl_mode_       = msg;
        mode_change_     = true;
        RCLCPP_INFO(node_->get_logger(), "[ctrltypeCallback] mode=%d, joint_move type=%d, dur=%.2f", msg, static_cast<int>(type), duration);
    }  

    //ctrl_update for wholebody motion
    void FrankaHuskyWrapper::ctrl_update(const int& msg, WBDMoveType type, const Vector3d& wbd_pos, double duration){
        wbd_move_type_ = type;
        wbd_pos_       = wbd_pos;
        wbd_duration_  = duration;
        ctrl_mode_     = msg;
        mode_change_   = true;
        RCLCPP_INFO(node_->get_logger(), "[ctrltypeCallback] mode=%d, wbd_move type=%d, dur=%.2f", msg, static_cast<int>(type), duration);
    }

    void FrankaHuskyWrapper::resetAllTasks() {
        tsid_->removeTask("task-mobile-ori");
        tsid_->removeTask("task-mobile-trans");
        tsid_->removeTask("task-se3");
        tsid_->removeTask("task-posture");
        tsid_->removeTask("task-torque-bounds");
    }

    void FrankaHuskyWrapper::initPostureTraj(double duration, JointMoveType type){
        VectorXd q_tail = ismobile_ ? state_.q_.tail(na_-2) : state_.q_.tail(na_);
        trajPosture_Cubic_->setInitSample(q_tail);
        trajPosture_Cubic_->setDuration(duration);
        trajPosture_Cubic_->setStartTime(time_);

        switch (type) {
            case JointMoveType::Home:  q_goal_ = q_ref_; break; //home posture
            case JointMoveType::Hold:  q_goal_ = q_tail; break; //stop at current posture for safe
        }
        trajPosture_Cubic_->setGoalSample(q_goal_);
    }

    void FrankaHuskyWrapper::initMobileTraj(double duration){
        H_mobile_ref_ = robot_->getMobilePosition(data_, 5);//5 means state.q_ at panda joint 1 
        trajMobile_Cubic_->setStartTime(time_);
        trajMobile_Cubic_->setDuration(duration);
        trajMobile_Cubic_->setInitSample(H_mobile_ref_);
        trajMobile_Cubic_->setGoalSample(H_mobile_ref_);
    }

    void FrankaHuskyWrapper::initEETraj(double duration){
        H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_; 
        eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
        trajEE_Cubic_->setStartTime(time_);
        trajEE_Cubic_->setDuration(duration);
        trajEE_Cubic_->setInitSample(H_ee_ref_);
    }

    void FrankaHuskyWrapper::updateMobile() {
        if (!ismobile_) return;
        trajMobile_Cubic_->setCurrentTime(time_);
        sampleMobile_ = trajMobile_Cubic_->computeNext();
        mobileTask_trans_->setReference(sampleMobile_);        
    }

    void FrankaHuskyWrapper::updateMobileOri(){
        if (!ismobile_) return;
        trajMobile_Cubic_->setCurrentTime(time_);
        sampleMobile_ = trajMobile_Cubic_->computeNext();
        mobileTask_ori_->setReference(sampleMobile_);
        mobileTask_trans_->setReference(sampleMobile_);
    }

    // ROS1의 mobile_path_client_.call(srv) 대체.
    // 전용 노드(srv_node_)만 스핀하므로 컨트롤러 노드의 실행자와 얽히지 않는다.
    // ROS1처럼 "호출 성공 여부"만 돌려주고, 경로가 비었는지는 호출부가 본다.
    bool FrankaHuskyWrapper::callMobilePathPlanner(){
        if (!mobile_path_client_->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_WARN(node_->get_logger(),
                "[path] service 'plan_mobile_path' unavailable (planner server not running?)");
            return false;
        }

        auto request = std::make_shared<PlanMobilePath::Request>(mobile_path_req_);
        auto future  = mobile_path_client_->async_send_request(request);

        if (rclcpp::spin_until_future_complete(srv_node_, future, std::chrono::seconds(10))
            != rclcpp::FutureReturnCode::SUCCESS){
            RCLCPP_WARN(node_->get_logger(), "[path] planner call timed out");
            return false;
        }

        mobile_path_res_ = *future.get();
        return true;
    }

    void FrankaHuskyWrapper::setMujocoObstacles(){
        // Obstacles are provided by the launch file (rosparam), so the controller stays
        // scene-independent: each scene's launch sets its own list, no rebuild needed.
        //   obstacle_margin : double        (optional, default 0.15)
        //   obstacles       : float64[4N]   flattened [cx, cy, half_x, half_y] per box
        //                                   (world frame, meters)
        // Each entry mirrors a MuJoCo box (center pos + half-extent size); it is stored as an
        // axis-aligned Obstacle2D rectangle (x1,y1)-(x2,y2). The planner ALREADY inflates by the
        // Husky footprint (robot_shape_source), so 'margin' is only extra clearance -- keep it
        // small or the start can fall inside the inflated obstacle -> "no valid initial states".
        mobile_path_req_.obstacles2d.clear();

        const double margin = declare_or_get<double>("obstacle_margin", 0.15);

        // ⚠ ROS2 파라미터는 1-D 배열만 된다. ROS1이 올리던 [[cx,cy,hx,hy], ...] 를
        // launch가 길이 4N으로 평탄화해 넘기므로 4개씩 끊어 읽는다.
        const vector<double> obs_flat = declare_or_get<vector<double>>("obstacles", {});
        if (obs_flat.empty()){
            RCLCPP_WARN_ONCE(node_->get_logger(),
                "[obstacles] no 'obstacles' param -> planning without obstacles");
            return;
        }
        if (obs_flat.size() % 4 != 0){
            RCLCPP_WARN(node_->get_logger(),
                "[obstacles] length %d is not a multiple of 4 (need [cx,cy,half_x,half_y] per box) -> ignored",
                (int)obs_flat.size());
            return;
        }

        for (size_t i = 0; i + 3 < obs_flat.size(); i += 4){
            const double cx = obs_flat[i],     cy = obs_flat[i + 1];
            const double hx = obs_flat[i + 2], hy = obs_flat[i + 3];
            icir_path_planner_ros_interface::msg::Obstacle2D o;
            o.x1.data = cx - hx - margin;  o.y1.data = cy - hy - margin;
            o.x2.data = cx + hx + margin;  o.y2.data = cy + hy + margin;
            mobile_path_req_.obstacles2d.push_back(o);
        }
        RCLCPP_INFO_ONCE(node_->get_logger(),
                         "[obstacles] loaded %d obstacle(s) from param (margin=%.2f)",
                         (int)mobile_path_req_.obstacles2d.size(), margin);
    }

    void FrankaHuskyWrapper::updatePosture() {
        trajPosture_Cubic_->setCurrentTime(time_);
        samplePosture_ = trajPosture_Cubic_->computeNext();
        postureTask_->setReference(samplePosture_);
    }
    
    void FrankaHuskyWrapper::updateEE() { 
        trajEE_Cubic_->setCurrentTime(time_);
        sampleEE_ = trajEE_Cubic_->computeNext();
        eeTask_->setReference(sampleEE_);
    }

    void FrankaHuskyWrapper::solveHQP() {  
        const HQPData & HQPData = tsid_->computeProblemData(time_, state_.q_, state_.v_);
        state_.torque_ = tsid_->getAccelerations(solver_->solve(HQPData));   
    }

    void FrankaHuskyWrapper::updateAndSolve(){
        bool hasOri = false;
        for (auto& task : tsid_->m_taskMotions){
            const std::string& name = task->task.name();
            if      (name == "task-mobile-ori")  { hasOri = true; updateMobileOri(); }
            else if (name == "task-mobile-trans" && !hasOri) updateMobile();
            else if (name == "task-posture")     updatePosture();
            else if (name == "task-se3")         updateEE();
        }
        solveHQP();
    }

    void FrankaHuskyWrapper::compute(const double& time){
        time_ = time;

        robot_->computeAllTerms(data_, state_.q_, state_.v_);
        // robot_->computeAllTerms_ABA(data_, state_.q_, state_.v_, state_.tau_); //to try to use data.ddq (only computed from ABA) However,the ddq value with ABA is not reasonalbe.        

        //---------------------------------------------------------------------------------------------------------//  
        //------------------------------------------ gravity motion -------------------------------------------------//  
        //---------------------------------------------------------------------------------------------------------//  
        if (ctrl_mode_ == 0){ //g // gravity mode
            state_.torque_.setZero();
        }
        
        //---------------------------------------------------------------------------------------------------------//  
        //------------------------------------------ posture motion -------------------------------------------------//  
        //---------------------------------------------------------------------------------------------------------//  
        if (ctrl_mode_ == 1){ //h //init position
            if (mode_change_){ 
                //remove               
                resetAllTasks();

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);    

                //posture  
                initPostureTraj(joint_duration_, joint_move_type_); 

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask_trans_, 1.0, 0);                    
                    mobileTask_trans_->setDesiredinertia(MatrixXd::Identity(6,6));       

                    initMobileTraj(joint_duration_);                        
                }         

                mode_change_ = false;             
            } 

            updateAndSolve();
        }  

        //---------------------------------------------------------------------------------------------------------//  
        //-------------------------------------- ee move w/o mobile------------------------------------------------//  
        //---------------------------------------------------------------------------------------------------------//   
        if (ctrl_mode_ == 2){ //EE move
            if (mode_change_){
                //remove      
                resetAllTasks();

                //add
                tsid_->addMotionTask(*postureTask_, 1e-16, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);

                //posture
                initPostureTraj(ee_jog_duration_, JointMoveType::Hold);

                //ee
                initEETraj(ee_jog_duration_);
                H_ee_ref_.translation()(ee_jog_axis_) += ee_jog_delta_;                
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask_trans_, 1.0, 0);                    
                    mobileTask_trans_->setDesiredinertia(MatrixXd::Identity(6,6));        

                    initMobileTraj(ee_jog_duration_);   
                }

                mode_change_ = false;                
            }

            updateAndSolve();
        }    

        //---------------------------------------------------------------------------------------------------------//  
        //-------------------------------------------- mobile jog -------------------------------------------------//  
        //---------------------------------------------------------------------------------------------------------//  
        if (ctrl_mode_ == 3){ //mobile jog mode
            if (mode_change_){
                //remove
                resetAllTasks();

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                //posture
                initPostureTraj(ee_jog_duration_, JointMoveType::Hold);

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask_ori_, 1.0, 1);
                    mobileTask_ori_->setDesiredinertia(MatrixXd::Identity(6,6));     

                    tsid_->addMotionTask(*mobileTask_trans_, 0.1, 1);
                    mobileTask_trans_->setDesiredinertia(MatrixXd::Identity(6,6));                         

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(ee_jog_duration_);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                    
                    H_mobile_ref_.translation()(ee_jog_axis_) += ee_jog_delta_;
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);     
                }

                mode_change_ = false;
                est_time_ = time_;
            }

            updateAndSolve();
            if (time_ > est_time_ + ee_jog_duration_ + 3.0) state_.torque_.head(2).setZero(); //3.0 is buffer time
        }   

        //---------------------------------------------------------------------------------------------------------//  
        //------------------------------------ whole body  motion -------------------------------------------------//  
        //---------------------------------------------------------------------------------------------------------//  
        if (ctrl_mode_ == 4){ //1 //test move: wholebody motion for 0.3x & -0.1z ee pos
            if (mode_change_){
                //remove         
                resetAllTasks();

                //add
                tsid_->addMotionTask(*postureTask_, 1e-16, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);

                //posture 
                initPostureTraj(wbd_duration_, JointMoveType::Hold);

                //ee
                initEETraj(wbd_duration_);
                switch (wbd_move_type_) {
                    case WBDMoveType::Absolute:        H_ee_ref_.translation()  = wbd_pos_; break;
                    case WBDMoveType::DeltaInGlobal:   H_ee_ref_.translation() += wbd_pos_; break;
                    case WBDMoveType::DeltaInLocal:    H_ee_ref_.translation() += H_ee_ref_.rotation() * wbd_pos_; break;
                }

                trajEE_Cubic_->setGoalSample(H_ee_ref_);
                
                mode_change_ = false;                
            }            

            updateAndSolve();          
        }

        //---------------------------------------------------------------------------------------------------------//  
        //----------------------------------------- estimation motion ---------------------------------------------//  
        //---------------------------------------------------------------------------------------------------------//      
        if (ctrl_mode_ == 11){ //w //identification motion w/o mobile motion, getcalibration available 
            if (mode_change_){
                //remove  
                resetAllTasks();

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);

                // //posture   
                initPostureTraj(2.0, JointMoveType::Home);

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_; 

                //mobility                
                if (ismobile_) { 
                    target_time_ = traj_length_in_time_;

                    tsid_->addMotionTask(*mobileTask_trans_, 1.0, 0);                    
                    mobileTask_trans_->setDesiredinertia(MatrixXd::Identity(6,6));        

                    initMobileTraj(target_time_);    
                }
               
                mode_change_ = false;
                trjectory_end_ = false;
                
                est_time_ = time_;
                T_vel_.setIdentity();

                if (getcalibration_){
                    std::string path = ament_index_cpp::get_package_share_directory("icir_phri_panda_husky") + "/calibration/calibration.txt";
                    fout_.open(path);

                }
            }

            updateMobile();
            updatePosture();                       
            ////////////////// 

            VectorXd vel_vec, vel_vec_pseudo, vel_vec_null, vel_vec_null_forpseudo;            
            vel_vec.resize(6);
            vel_vec_pseudo.resize(3);   
            vel_vec_null.resize(6);                                
            vel_vec_null_forpseudo.resize(6);                                           
            if (time_ - est_time_ < target_time_){  
                //for pseudo motion----------------------------------------------------------//                
                vel_vec_pseudo << 0.0, 0.0, 0.0;                

                //vel_vec_pseudo should be used here to eliminate rx motion in pseudo motion. see the relation of matrix hGr_local_pinv_ and hGr_local_Null_
                vel_vec_null_forpseudo << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;                  
                //---------------------------------------------------------------------------//                

                //for null motion------------------------------------------------------------//
                if (time_ - est_time_ < traj_length_in_time_){                                        
                    double fx = 0.4;                                
                    double fy = 0.4;                                
                    double fz = 0.4;                                
                    if (isrobotiq_) { 
                        //to make global -x axis (5deg) & -y axis (3deg)
                        double anglex = -8*M_PI/180.0*2.0*M_PI*fx*cos(2.0*M_PI*fx*(time_ - est_time_));
                        double angley = +5*M_PI/180.0*2.0*M_PI*fy*cos(2.0*M_PI*fy*(time_ - est_time_));
                        double anglez = -3*M_PI/180.0*2.0*M_PI*fz*cos(2.0*M_PI*fz*(time_ - est_time_));                        
                        // double anglez = 0.0;                        
                    
                        vel_vec_null << 0.0, 0.0, 0.0, anglex, angley, anglez; //joint7 is 90deg rotated w.r.t. global coordinate
                    }
                    else {
                        //to make global -x axis (5deg) & -y axis (3deg)
                        double anglex = -5*M_PI/180.0*2.0*M_PI*fx*cos(2.0*M_PI*fx*(time_ - est_time_)) -3*M_PI/180.0*2.0*M_PI*fy*cos(2.0*M_PI*fy*(time_ - est_time_));
                        double angley = -5*M_PI/180.0*2.0*M_PI*fx*cos(2.0*M_PI*fx*(time_ - est_time_)) +3*M_PI/180.0*2.0*M_PI*fy*cos(2.0*M_PI*fy*(time_ - est_time_));                                            
                        
                        vel_vec_null << 0.0, 0.0, 0.0, anglex, angley, 0.0;
                    }

                    if (getcalibration_){
                    fout_ << Fext_.transpose() << "\n";
                    }                                
                }
                else { //when estimation is finished
                    if (!trjectory_end_){                    
                        trjectory_end_ = true;
                        
                        if (getcalibration_) {
                            fout_.close();          
                        }
                    }
                    vel_vec_null.setZero();
                }
                //---------------------------------------------------------------------------//                
                    
                vel_vec = hGr_local_pinv_ * vel_vec_pseudo + hGr_local_Null_ * (vel_vec_null_forpseudo + vel_vec_null);                
            }
            else{  //when pseudo motion is end                
                // maintain zero velocity after the trajectory is finished to avoid hard stop -----------//                                                
                vel_vec.setZero();                                
                // ------------------------------------------------------------------------------------//
            }

            T_vel_ = T_vel_ * vel_to_SE3(vel_vec, dt_);   //velocity integration to make position
            pinocchio::SE3 H_EE_ref_estimation;
            H_EE_ref_estimation = H_ee_ref_;                
            H_EE_ref_estimation = H_EE_ref_estimation * T_vel_;                
            SE3ToVector(H_EE_ref_estimation, sampleEE_.pos);

            eeTask_->setReference(sampleEE_);
            //////////////////

            solveHQP(); 
        }                   

        if (ctrl_mode_ == 12){ //e //identification motion w/ mobile motion 
            if (mode_change_){
                //remove  
                resetAllTasks();

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);

                //posture
                initPostureTraj(2.0, JointMoveType::Home);

                //ee
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                H_ee_ref_ = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;

                //mobility                
                if (ismobile_) {                    
                    target_distance_ = 1.5;
                    target_time_ = 15.0;
                    // target_velocity_ = target_distance_ / target_time_;                                        
                    
                    //*----- Use of mobileTask and eeTask together diverge (why?) -----*//
                    // tsid_->addMotionTask(*mobileTask_ori_, 1.0, 1);                                        
                    // mobileTask_ori_->setDesiredinertia(MatrixXd::Identity(6,6));     

                    tsid_->addMotionTask(*mobileTask_trans_, 0.1, 1);
                    mobileTask_trans_->setDesiredinertia(MatrixXd::Identity(6,6));                         

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(target_time_);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_); 

                    H_mobile_ref_.translation()(0) += target_distance_;                   
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);  
                }
               
                mode_change_ = false;
                trjectory_end_ = false;
                
                est_time_ = time_;
                T_vel_.setIdentity();                
            }

            updateMobile();
            updatePosture();                        
            ////////////////// 

            VectorXd vel_vec, vel_vec_pseudo, vel_vec_null, vel_vec_null_forpseudo;            
            vel_vec.resize(6);
            vel_vec_pseudo.resize(3);   
            vel_vec_null.resize(6);                                
            vel_vec_null_forpseudo.resize(6);                                           
            if (time_ - est_time_ < target_time_){  
                //for pseudo motion----------------------------------------------------------//                
                //constant velocity
                // vel_vec_pseudo  = R_joint7_atHome_ * Vector3d(target_velocity_, 0.0, 0.0); // -0.01 m/s in global x axis
                
                //cubic velocity
                double a1, a2, vel_tra;
                a1 = +6.0 / pow(target_time_, 2) * target_distance_;
                a2 = -6.0 / pow(target_time_, 3) * target_distance_;
                vel_tra = a1 * (time_ - est_time_) + a2 * pow(time_ - est_time_, 2);
                vel_vec_pseudo  = R_joint7_atHome_ * Vector3d(vel_tra, 0.0, 0.0); // -0.01 m/s in global x axis                

                //vel_vec_pseudo should be used here to eliminate rx motion in pseudo motion. see the relation of matrix hGr_local_pinv_ and hGr_local_Null_
                vel_vec_null_forpseudo << vel_vec_pseudo(0), vel_vec_pseudo(1), 0.0, 0.0, 0.0, 0.0;                  
                //---------------------------------------------------------------------------//                

                //for null motion------------------------------------------------------------//
                if (time_ - est_time_ < traj_length_in_time_){                                        
                    double fx = 0.4;                                
                    double fy = 0.4;                                
                    double fz = 0.4;                                
                    if (isrobotiq_) { 
                        //to make global -x axis (5deg) & -y axis (3deg)
                        double anglex = -8*M_PI/180.0*2.0*M_PI*fx*cos(2.0*M_PI*fx*(time_ - est_time_));
                        double angley = +5*M_PI/180.0*2.0*M_PI*fy*cos(2.0*M_PI*fy*(time_ - est_time_));
                        double anglez = -3*M_PI/180.0*2.0*M_PI*fz*cos(2.0*M_PI*fz*(time_ - est_time_));                        
                        // double anglez = 0.0;                        
                    
                        vel_vec_null << 0.0, 0.0, 0.0, anglex, angley, anglez;
                    }
                    else {
                        //to make global -x axis (5deg) & -y axis (3deg)
                        double anglex = -5*M_PI/180.0*2.0*M_PI*fx*cos(2.0*M_PI*fx*(time_ - est_time_)) -3*M_PI/180.0*2.0*M_PI*fy*cos(2.0*M_PI*fy*(time_ - est_time_));
                        double angley = -5*M_PI/180.0*2.0*M_PI*fx*cos(2.0*M_PI*fx*(time_ - est_time_)) +3*M_PI/180.0*2.0*M_PI*fy*cos(2.0*M_PI*fy*(time_ - est_time_));                                            
                        
                        vel_vec_null << 0.0, 0.0, 0.0, anglex, angley, 0.0;
                    }                    
                }
                else { //when estimation is finished
                    if (!trjectory_end_){                    
                        trjectory_end_ = true;                                             
                    }
                    vel_vec_null.setZero();
                }
                //---------------------------------------------------------------------------//                
                    
                vel_vec = hGr_local_pinv_ * vel_vec_pseudo + hGr_local_Null_ * (vel_vec_null_forpseudo + vel_vec_null);                
            }
            else{  //when pseudo motion is end                
                // maintain zero velocity after the trajectory is finished to avoid hard stop -----------//                                                
                vel_vec.setZero();                                
                // ------------------------------------------------------------------------------------//
            }

            T_vel_ = T_vel_ * vel_to_SE3(vel_vec, dt_);   //velocity integration to make position
            pinocchio::SE3 H_EE_ref_estimation;
            H_EE_ref_estimation = H_ee_ref_;                
            H_EE_ref_estimation = H_EE_ref_estimation * T_vel_;                
            SE3ToVector(H_EE_ref_estimation, sampleEE_.pos);

            eeTask_->setReference(sampleEE_);
            //////////////////

            solveHQP();            
        }           

        //---------------------------------------------------------------------------------------------------------//  
        //------------------------------------------- control motion ----------------------------------------------//  
        //---------------------------------------------------------------------------------------------------------//   
        if (ctrl_mode_ == 21){ //c //admittance control
            if (mode_change_){
                //remove      
                resetAllTasks();

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);

                //posture 
                initPostureTraj(2.0, JointMoveType::Home);

                //ee
                initEETraj(wbd_duration_);
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask_trans_, 1.0, 0);                    
                    mobileTask_trans_->setDesiredinertia(MatrixXd::Identity(6,6));        

                    initMobileTraj(2.0); 
                }
               
                mode_change_ = false;
                
                est_time_ = time_;      
                initial_calibration_update_ = false;          
                
                //damping gain for addmittance control
                Dinv_gain_ << 0.01, 0.01, 0.01, 0.01, 0.01, 0.03; 
                Dinv_gain_ *= dt_;
            }

            updateMobile();
            updatePosture(); 
            //////////////////   
            if ((!initial_calibration_update_) && (time_ - est_time_ > 0.5)){ //update calibration after controller runs
                Fext_calibration_ = Fext_; //Fext is in GLOBAL
                initial_calibration_update_ = true;
                cout << "initial ext_calibration_" << endl;
                cout << Fext_calibration_.transpose() << endl;            

                bool a[6] = {false};
                for (int i=0; i<6; i++) {
                    a[i] = Dinv_gain_(i) * eeTask_->Kp()(i) < 1.0;
                }                
                cout << "Kf gain verification" << endl;
                cout << a[0] << "   "  << a[1] << "   "  << a[2] << "   "  << a[3] << "   "  << a[4] << "   "  << a[5] << endl;                                                                                
            }
            if (initial_calibration_update_) {                                                                                                                
                SE3 m_wMl;                
                m_wMl = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;                                                
                m_wMl.translation() << 0.0, 0.0, 0.0;
                
                Force Fext_global;
                Fext_global = Fext_ - Fext_calibration_;

                Vector3d offset_vec_global, offset_vec_local;
                // offset_vec_global(0) =  Dinv_gain_(5) * noise_elimination(Fext_global.angular()(2), 0.3) * 0.0;
                // offset_vec_global(1) =  Dinv_gain_(1) * noise_elimination(Fext_global.linear()(1), 1.0) * 0.0;
                // offset_vec_global(2) = -Dinv_gain_(3) * noise_elimination(Fext_global.angular()(0), 0.0);                

                offset_vec_global(0) =  Dinv_gain_(0) * noise_elimination(Fext_global.linear()(0), 1.0);
                offset_vec_global(1) =  Dinv_gain_(1) * noise_elimination(Fext_global.linear()(1), 1.0);
                offset_vec_global(2) =  Dinv_gain_(2) * noise_elimination(Fext_global.linear()(2), 1.0);      

                offset_vec_local = m_wMl.actInv(offset_vec_global); //global to local                
                
                SE3 Fext_offset;
                Fext_offset.setIdentity();
                Fext_offset.translation() = offset_vec_local;  
                //////////////////  

                H_ee_ref_ = H_ee_ref_ * Fext_offset; //integration manner                                                                                                                             
                
                ////////////////////////////////////////////////////////
                // offset_vec_local.setZero();
                // offset_vec_local(2) = Dinv_gain_(3) * noise_elimination((Fext_-Fext_calibration_)(3), 1.0);                
                // H_ee_ref_.translation() += -offset_vec_local; //integration manner                                                                             

                // // calibration update --> hard to use, if position is changed a lot, the use of f button strategy is better than calibration update
                // if (fabs(Fext_offset.translation()(2)) < 1e-5) {
                //     Fext_calibration_ = Fext_;
                //     cout << "updated Fext_calibration_" << endl;
                //     cout << Fext_calibration_.transpose() << endl;            
                // } 
            }            
           
            SE3ToVector(H_ee_ref_, sampleEE_.pos);

            eeTask_->setReference(sampleEE_);
            //////////////////

            solveHQP();
        } 

        if (ctrl_mode_ == 22){ //v //impedance control
            if (mode_change_){
                //remove      
                resetAllTasks();

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);

                //posture
                initPostureTraj(2.0, JointMoveType::Home);

                //ee
                initEETraj(wbd_duration_);
                trajEE_Cubic_->setGoalSample(H_ee_ref_);   

                //mobility
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask_trans_, 1.0, 0);                    
                    mobileTask_trans_->setDesiredinertia(MatrixXd::Identity(6,6));        

                    initMobileTraj(2.0);   
                }
               
                mode_change_ = false;                                                
            }            

            updateMobile();
            updatePosture(); 
            //////////////////                        
            
            if (1) { 
                //to make K(x-xd)=0, put xd=x 
                H_ee_ref_.translation() = (robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_).translation();
                
                //inertia shaping    
                // Me_inv_ = 
                eeTask_->setDesiredinertia(Me_inv_);
            }
            else {   
                //to make K(x-xd)=0, put K=0 
                Vector6d a;
                a = eeTask_->Kp();
                a.head(3) << 0.0, 0.0, 0.0;
                eeTask_->Kp(a); //after this task, Kp should be returned to original gain
            }                        

            SE3ToVector(H_ee_ref_, sampleEE_.pos);

            eeTask_->setReference(sampleEE_);
            //////////////////

            solveHQP();
        }                               

        //---------------------------------------------------------------------------------------------------------//  
        //------------------------------------------- test motion -------------------------------------------------//  
        //---------------------------------------------------------------------------------------------------------// 
        if (ctrl_mode_ == 101){ //3 //test move: chicken head motion with +0.1m mobile motion
            if (mode_change_){
                //remove
                resetAllTasks();

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);

                //posture
                initPostureTraj(2.0, JointMoveType::Home);

                //ee
                initEETraj(wbd_duration_);
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility                
                if (ismobile_) {
                    tsid_->addMotionTask(*mobileTask_ori_, 1.0, 0);
                    mobileTask_ori_->setDesiredinertia(MatrixXd::Identity(6,6));     

                    tsid_->addMotionTask(*mobileTask_trans_, 1.0, 0);
                    mobileTask_trans_->setDesiredinertia(MatrixXd::Identity(6,6));     

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(2.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                    
                    H_mobile_ref_.translation()(0) += 0.1;
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);     
                }

                mode_change_ = false;
                est_time_ = time_;
            }

            updateAndSolve();
            if (time_ > est_time_ + 2.0 + 3.0) state_.torque_.head(2).setZero();
        }  

        if (ctrl_mode_ == 102){ //4 //test move: chicken head motion with -0.1m mobile motion
            if (mode_change_){
                //remove
                resetAllTasks();

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);

                //posture
                initPostureTraj(2.0, JointMoveType::Home);

                //ee
                initEETraj(wbd_duration_);
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask_ori_, 1.0, 0);
                    mobileTask_ori_->setDesiredinertia(MatrixXd::Identity(6,6));     

                    tsid_->addMotionTask(*mobileTask_trans_, 1.0, 0);
                    mobileTask_trans_->setDesiredinertia(MatrixXd::Identity(6,6));                         

                    H_mobile_ref_ = robot_->getMobilePosition(data_, 5); //5 means state.q_ at panda joint 1 
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(2.0);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                    
                    H_mobile_ref_.translation()(0) -= 0.1;
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);     
                }

                mode_change_ = false;
                est_time_ = time_;
            }

            updateAndSolve();
            if (time_ > est_time_ + 2.0 + 3.0) state_.torque_.head(2).setZero();
        }  

        if (ctrl_mode_ == 103){ //5 //test move: mobile base backward during 5sec, maintaining posture        
            if (mode_change_){
                //remove
                resetAllTasks();

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);

                //posture               
                initPostureTraj(2.0, JointMoveType::Home);            

                mode_change_ = false;
                est_time_ = time_;
            }
            // husky
            if (est_time_ + 5.0 > time_) {                
                state_.torque_(0) = -50.;
                state_.torque_(1) = -50.;                
            }          
            else{
                state_.torque_(0) = 0.0;
                state_.torque_(1) = 0.0;                
            }

            updatePosture(); 
            solveHQP();         
        }

        if (ctrl_mode_ == 104){ //test move: path planner based mobile motion with SE3 maintenance
            if (mode_change_){
                resetAllTasks();

                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 1);
                eeTask_->setDesiredinertia(MatrixXd::Identity(6,6));
                if (ismobile_) {
                    tsid_->addMotionTask(*mobileTask_ori_, 0.1, 1);
                    mobileTask_ori_->setDesiredinertia(MatrixXd::Identity(6,6));
                    tsid_->addMotionTask(*mobileTask_trans_, 1.0, 1);
                    mobileTask_trans_->setDesiredinertia(MatrixXd::Identity(6,6));
                }

                initPostureTraj(1.0, JointMoveType::Hold);
                initEETraj(1.0);
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                // store initial EE-mobile offset (global frame) and initial EE orientation
                {
                    pinocchio::SE3 H_mob_init = robot_->getMobilePosition(data_, 5);
                    T_vel_.translation() = H_ee_ref_.translation() - H_mob_init.translation();
                    T_vel_.rotation()    = H_ee_ref_.rotation();
                }

                // request path from planner
                H_mobile_ref_ = robot_->getMobilePosition(data_, 5);
                double cur_theta = mobileYaw(H_mobile_ref_);

                mobile_path_req_.current_mobile_state.x     = H_mobile_ref_.translation()(0);
                mobile_path_req_.current_mobile_state.y     = H_mobile_ref_.translation()(1);
                mobile_path_req_.current_mobile_state.theta = cur_theta;
                mobile_path_req_.target_mobile_pose.x       = H_mobile_ref_.translation()(0) + 2.3;
                mobile_path_req_.target_mobile_pose.y       = H_mobile_ref_.translation()(1) - 1.5;
                mobile_path_req_.target_mobile_pose.theta   = cur_theta - 0.707;
                // mobile_path_req_.obstacles2d.clear();
                setMujocoObstacles();

                RCLCPP_INFO(node_->get_logger(), "[104] request: start(%.2f, %.2f, %.2f) -> goal(%.2f, %.2f, %.2f)",
                    mobile_path_req_.current_mobile_state.x,
                    mobile_path_req_.current_mobile_state.y,
                    mobile_path_req_.current_mobile_state.theta,
                    mobile_path_req_.target_mobile_pose.x,
                    mobile_path_req_.target_mobile_pose.y,
                    mobile_path_req_.target_mobile_pose.theta);

                if (callMobilePathPlanner() && !mobile_path_res_.mobile_path.points.empty()){
                    node_num_ = (double)(mobile_path_res_.mobile_path.points.size()) - 1.0;
                    auto& first = mobile_path_res_.mobile_path.points.front();
                    auto& last  = mobile_path_res_.mobile_path.points.back();
                    RCLCPP_INFO(node_->get_logger(), "[104] Path received: %d waypoints, first(%.2f,%.2f) last(%.2f,%.2f)",
                        (int)node_num_+1, first.x, first.y, last.x, last.y);
                } else {
                    node_num_ = -1;
                    RCLCPP_WARN(node_->get_logger(), "Path planner call failed");
                }

                if (ismobile_) {
                    trajMobile_Cubic_->setStartTime(time_);
                    trajMobile_Cubic_->setDuration(0.001);
                    trajMobile_Cubic_->setInitSample(H_mobile_ref_);
                    trajMobile_Cubic_->setGoalSample(H_mobile_ref_);
                }

                mode_change_          = false;
                fine_adjust_init_        = false;
                mobile_path_succeed_  = false;
                stime_                = time_;
                prev_node_            = -1;
                node_index_           = 0;
            }

            if (ismobile_ && node_num_ != -1){
                int idx = (int)node_index_;
                auto& pt = mobile_path_res_.mobile_path.points[idx];

                setPlanarPose(H_mobile_ref_, pt.x, pt.y, pt.theta);

                if (node_index_ < node_num_){
                    if ((int)prev_node_ != idx){
                        trajMobile_Cubic_->setStartTime(time_);
                        trajMobile_Cubic_->setDuration(0.001);
                        trajMobile_Cubic_->setInitSample(robot_->getMobilePosition(data_, 5));
                        trajMobile_Cubic_->setGoalSample(H_mobile_ref_);

                        // EE reference follows mobile: fixed global orientation, position offset constant
                        H_ee_ref_.translation() = H_mobile_ref_.translation() + T_vel_.translation();
                        H_ee_ref_.rotation()    = T_vel_.rotation();
                        trajEE_Cubic_->setStartTime(time_);
                        trajEE_Cubic_->setDuration(0.001);
                        trajEE_Cubic_->setInitSample(robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_);
                        trajEE_Cubic_->setGoalSample(H_ee_ref_);

                        prev_node_ = (double)idx;
                        stime_     = time_;
                    }
                    if (stime_ + 0.1 <= time_)
                        node_index_ += 1.0;
                } else {
                    if (!fine_adjust_init_){
                        tsid_->updateTaskWeight("task-mobile-ori", 1.);
                        tsid_->updateTaskWeight("task-mobile-trans", 0.01);

                        trajMobile_Cubic_->setStartTime(time_);
                        trajMobile_Cubic_->setDuration(1.5);
                        trajMobile_Cubic_->setInitSample(robot_->getMobilePosition(data_, 5));
                        trajMobile_Cubic_->setGoalSample(H_mobile_ref_);

                        // EE reference follows mobile: fixed global orientation, position offset constant
                        H_ee_ref_.translation() = H_mobile_ref_.translation() + T_vel_.translation();
                        H_ee_ref_.rotation()    = T_vel_.rotation();

                        trajEE_Cubic_->setStartTime(time_);
                        trajEE_Cubic_->setDuration(1.5);
                        trajEE_Cubic_->setInitSample(robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_);
                        trajEE_Cubic_->setGoalSample(H_ee_ref_);

                        fine_adjust_init_ = true;
                    }

                    if (!mobile_path_succeed_){
                        pinocchio::SE3 cur = robot_->getMobilePosition(data_, 5);
                        double cur_th = mobileYaw(cur);
                        double ref_th = mobileYaw(H_mobile_ref_);
                        double th_err = fabs(cur_th - ref_th);
                        if (th_err > M_PI) th_err = 2*M_PI - th_err;
                        if (th_err < 0.05){
                            mobile_path_succeed_ = true;
                            RCLCPP_INFO(node_->get_logger(), "[104] Path complete. theta_err: %.3f rad -> holding position", th_err);
                        }
                    } 
                }
            }

            if (!mobile_path_succeed_) {
                pinocchio::SE3 mb_ = robot_->getMobilePosition(data_, 5);
                double th_ = mobileYaw(mb_);
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "[104] node: %d / %d, pos: %.2f %.2f theta: %.2f",
                    (int)node_index_, (int)node_num_,
                    mb_.translation()(0), mb_.translation()(1), th_);
            }

            updateAndSolve();
            if (mobile_path_succeed_) state_.torque_.head(2).setZero();
        }

        if (ctrl_mode_ == 105){ //test move: path planner based mobile-only motion with joint posture maintenance
            if (mode_change_){
                resetAllTasks();

                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                if (ismobile_) {
                    tsid_->addMotionTask(*mobileTask_ori_, 0.1, 1);
                    mobileTask_ori_->setDesiredinertia(MatrixXd::Identity(6,6));
                    tsid_->addMotionTask(*mobileTask_trans_, 1.0, 1);
                    mobileTask_trans_->setDesiredinertia(MatrixXd::Identity(6,6));
                }

                initPostureTraj(1.0, JointMoveType::Hold);

                // request path from planner
                H_mobile_ref_ = robot_->getMobilePosition(data_, 5);
                double cur_theta = mobileYaw(H_mobile_ref_);

                mobile_path_req_.current_mobile_state.x     = H_mobile_ref_.translation()(0);
                mobile_path_req_.current_mobile_state.y     = H_mobile_ref_.translation()(1);
                mobile_path_req_.current_mobile_state.theta = cur_theta;
                // Dubins is forward-only and the tables sit ahead/left, so aim the goal to the
                // OPEN right side (-y) to avoid the forward-left loop that clips the tables.
                mobile_path_req_.target_mobile_pose.x       = H_mobile_ref_.translation()(0) + 1.3;
                mobile_path_req_.target_mobile_pose.y       = H_mobile_ref_.translation()(1) + 2.5;
                mobile_path_req_.target_mobile_pose.theta   = cur_theta - 0.707;
                // mobile_path_req_.obstacles2d.clear();
                setMujocoObstacles();

                if (callMobilePathPlanner() && !mobile_path_res_.mobile_path.points.empty()){
                    node_num_ = (double)(mobile_path_res_.mobile_path.points.size()) - 1.0;
                } else {
                    node_num_ = -1;
                }

                mode_change_          = false;
                fine_adjust_init_        = false;
                mobile_path_succeed_  = false;
                stime_                = time_;
                prev_node_            = -1;
                node_index_           = 0;

                // logs
                RCLCPP_INFO(node_->get_logger(), "[105] request: start(%.2f, %.2f, %.2f) -> goal(%.2f, %.2f, %.2f)",
                    mobile_path_req_.current_mobile_state.x,
                    mobile_path_req_.current_mobile_state.y,
                    mobile_path_req_.current_mobile_state.theta,
                    mobile_path_req_.target_mobile_pose.x,
                    mobile_path_req_.target_mobile_pose.y,
                    mobile_path_req_.target_mobile_pose.theta);
                if (node_num_ != -1){
                    auto& first = mobile_path_res_.mobile_path.points.front();
                    auto& last  = mobile_path_res_.mobile_path.points.back();
                    RCLCPP_INFO(node_->get_logger(), "[105] Path received: %d waypoints, first(%.2f,%.2f) last(%.2f,%.2f)",
                        (int)node_num_+1, first.x, first.y, last.x, last.y);
                } else {
                    RCLCPP_WARN(node_->get_logger(), "[105] Path planner call failed");
                }
            } //mode_change finished

            if (ismobile_ && node_num_ != -1){
                int idx = (int)node_index_;
                auto& pt = mobile_path_res_.mobile_path.points[idx];

                setPlanarPose(H_mobile_ref_, pt.x, pt.y, pt.theta);

                if (node_index_ < node_num_){
                    if ((int)prev_node_ != idx){
                        prev_node_ = (double)idx;
                        stime_     = time_;
                    }
                    if (stime_ + 0.1 <= time_)
                        node_index_ += 1.0;
                } else {
                    if (!fine_adjust_init_){
                        tsid_->updateTaskWeight("task-mobile-ori", 1.);
                        tsid_->updateTaskWeight("task-mobile-trans", 0.01);

                        trajMobile_Cubic_->setStartTime(time_);
                        trajMobile_Cubic_->setDuration(1.5);
                        trajMobile_Cubic_->setInitSample(robot_->getMobilePosition(data_, 5));
                        trajMobile_Cubic_->setGoalSample(H_mobile_ref_);
                        fine_adjust_init_ = true;
                    }
                    
                    if (!mobile_path_succeed_){
                        pinocchio::SE3 cur = robot_->getMobilePosition(data_, 5);
                        double cur_th = mobileYaw(cur);
                        double ref_th = mobileYaw(H_mobile_ref_);
                        double th_err = fabs(cur_th - ref_th);
                        if (th_err > M_PI) th_err = 2*M_PI - th_err;
                        if (th_err < 0.05)
                            mobile_path_succeed_ = true;
                    }
                }

               
            } //   if (ismobile_ && node_num_ != -1)

            if (ismobile_ && node_num_ != -1 && node_index_ >= node_num_) {
                updateMobileOri();
            } else {
                SE3ToVector(H_mobile_ref_, sampleMobile_.pos);
                mobileTask_ori_->setReference(sampleMobile_);
                mobileTask_trans_->setReference(sampleMobile_);
            }
            updatePosture();
            solveHQP();

            // logs
            if (mobile_path_succeed_){
                state_.torque_.head(2).setZero();
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000, "[105] Path complete, holding position");
            } else {
                pinocchio::SE3 mb_ = robot_->getMobilePosition(data_, 5);
                double th_ = mobileYaw(mb_);
                RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 500, "[105] node: %d / %d, pos: %.2f %.2f theta: %.2f",
                    (int)node_index_, (int)node_num_,
                    mb_.translation()(0), mb_.translation()(1), th_);
            }
        }

        if (ctrl_mode_ == 99){ //p //print current ee state
            if (mode_change_){
                //remove     
                resetAllTasks();

                //add
                tsid_->addMotionTask(*postureTask_, 1e-6, 1);
                tsid_->addMotionTask(*torqueBoundsTask_, 1.0, 0);
                tsid_->addMotionTask(*eeTask_, 1.0, 0);

                //posture
                initPostureTraj(2.0, JointMoveType::Hold);

                //ee
                initEETraj(wbd_duration_);
                trajEE_Cubic_->setGoalSample(H_ee_ref_);

                cout << "current EE" << endl;
                cout << H_ee_ref_ << endl;

                //mobility                
                if (ismobile_) {                    
                    tsid_->addMotionTask(*mobileTask_trans_, 1.0, 0);
                    mobileTask_trans_->setDesiredinertia(MatrixXd::Identity(6,6));     

                    initMobileTraj(2.0); 
                }

                mode_change_ = false;                              
            }

            updateAndSolve();  
        }
    }        

    void FrankaHuskyWrapper::franka_output(VectorXd & qacc) { //from here to main code
        if (ismobile_) qacc = state_.torque_.tail(na_-2);
        else qacc = state_.torque_.tail(na_);
    }  
    void FrankaHuskyWrapper::husky_output(VectorXd & qvel) {
        qvel = state_.torque_.head(2);
    }  

    void FrankaHuskyWrapper::com(Eigen::Vector3d & com){
        //API:Vector of subtree center of mass positions expressed in the root joint of the subtree. 
        //API:In other words, com[j] is the CoM position of the subtree supported by joint j and expressed in the joint frame .         
        //API:The element com[0] corresponds to the center of mass position of the whole model and expressed in the global frame.
        com = robot_->com(data_);
    }

    void FrankaHuskyWrapper::position(pinocchio::SE3 & oMi){
        //API:Vector of absolute joint placements (wrt the world).
        oMi = robot_->position(data_, robot_->model().getJointId("panda_joint7"));
    }

    void FrankaHuskyWrapper::position_offset(pinocchio::SE3 & oMi){
        //API:Vector of absolute joint placements (wrt the world).
        oMi = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;
    }

    void FrankaHuskyWrapper::position_link0_offset(pinocchio::SE3 & oMi){
        //API:Vector of absolute joint placements (wrt the world).
        SE3 panda_link0_ee;
        panda_link0_ee = robot_->framePosition(data_, robot_->model().getFrameId("panda_link0"));

        SE3 panda_joint7_offset;
        this->position_offset(panda_joint7_offset);
        
        oMi = panda_link0_ee.inverse() * panda_joint7_offset;    
    }

    void FrankaHuskyWrapper::velocity(pinocchio::Motion & vel){
        //API:Vector of joint velocities expressed at the centers of the joints.
        vel = robot_->velocity(data_, robot_->model().getJointId("panda_joint7"));

        // NOTES ////////////////////////////////////////////////////////////////////////////
        // data.v = f.placement.actInv(data.v[f.parent]), both are in LOCAL coordinate
        // T_offset.act(v_frame) is WRONG method, It means that applying offset w.r.t. global coord. to EE frame.                
        //////////////////////////////////////////////////////////////////////////////////////

        // code comparison ///////////////////////////        
        // To apply offset, Adj_mat should be used
        //////////////////////////////////////////////                                
        // cout << "data.v" << endl;
        // cout <<  robot_->velocity(data_, robot_->model().getJointId("panda_joint7")) << endl;        

        // cout << "data.v with Adj_mat_" << endl;
        // cout << vel.linear() + Adj_mat_.topRightCorner(3,3) * vel.angular() << endl;
        // cout << vel.angular() << endl;        
    }

    void FrankaHuskyWrapper::velocity_offset(pinocchio::Motion & vel){
        //API:Vector of joint velocities expressed at the centers of the joints.
        vel = robot_->velocity(data_, robot_->model().getJointId("panda_joint7"));
        
        vel.linear() = vel.linear() + Adj_mat_.topRightCorner(3,3) * vel.angular();
        vel.angular() = vel.angular(); 
    }

    void FrankaHuskyWrapper::velocity_origin(pinocchio::Motion & vel){
        //API:Vector of joint velocities expressed at the origin. (data.ov)
        //Same with "vel = m_wMl.act(v_frame);"
        vel = robot_->velocity_origin(data_, robot_->model().getJointId("panda_joint7"));
    }

    void FrankaHuskyWrapper::acceleration(pinocchio::Motion & accel){        
        //API:Vector of joint accelerations expressed at the centers of the joints frames. (data.a)
        accel = robot_->acceleration(data_, robot_->model().getJointId("panda_joint7"));                                        
    }

    void FrankaHuskyWrapper::acceleration_origin(pinocchio::Motion & accel){
        //API:Vector of joint accelerations expressed at the origin of the world. (data.oa)        
        //It is not available!!!!!!!!!!!!!!! (always zero), so acceleratioon_global is used.
        accel = robot_->acceleration_origin(data_, robot_->model().getJointId("panda_joint7"));
    }

    void FrankaHuskyWrapper::acceleration_origin2(pinocchio::Motion & accel){
        //It is defined becuase data.oa is not available (output always zero) 
        SE3 m_wMl;
        Motion a_frame;        
        robot_->framePosition(data_, robot_->model().getFrameId("panda_joint7"), m_wMl);       //data.oMi     
        robot_->frameAcceleration(data_, robot_->model().getFrameId("panda_joint7"), a_frame); //data.a         
        accel = m_wMl.act(a_frame);
    }

    void FrankaHuskyWrapper::force(pinocchio::Force & force){
        //API:Vector of body forces expressed in the local frame of the joint. 
        //API:For each body, the force represents the sum of all external forces acting on the body.
        force = robot_->force(data_, robot_->model().getJointId("panda_joint7"));
    }

    void FrankaHuskyWrapper::force_origin(pinocchio::Force & force){
        //API:Vector of body forces expressed in the world frame. 
        //API:For each body, the force represents the sum of all external forces acting on the body.        
        //It is not available!!!!!!!!!!!!!!! (always zero), so acceleratioon_global is used.
        force = robot_->force_origin(data_, robot_->model().getJointId("panda_joint7"));
    }

    void FrankaHuskyWrapper::force_origin2(pinocchio::Force & force){
        //It is defined becuase data.of is not available (output always zero)         
        SE3 m_wMl;
        Force f_frame;
        robot_->framePosition(data_, robot_->model().getFrameId("panda_joint7"), m_wMl);
        robot_->frameForce(data_, robot_->model().getFrameId("panda_joint7"), f_frame);
        force = m_wMl.act(f_frame);
    }

    void FrankaHuskyWrapper::tau(VectorXd & tau_vec){
        //API:Vector of joint torques (dim model.nv).
        //It is not available (output always zero)
        if (ismobile_) tau_vec = robot_->jointTorques(data_).tail(na_-2);
        else           tau_vec = robot_->jointTorques(data_).tail(na_);
    }

    void FrankaHuskyWrapper::ddq(VectorXd & ddq_vec){
        //API:The joint accelerations computed from ABA.
        //It is not available (output always zero), will be available with ABA method
        //even though with ABA method, the value is not reasonable (behave like torque, not ddq)
        if (ismobile_) ddq_vec = robot_->jointAcceleration(data_).tail(na_-2);
        else           ddq_vec = robot_->jointAcceleration(data_).tail(na_);
    }

    void FrankaHuskyWrapper::mass(MatrixXd & mass_mat){
        if (ismobile_) mass_mat = robot_->mass(data_).bottomRightCorner(na_-2, na_-2);
        else           mass_mat = robot_->mass(data_).bottomRightCorner(na_, na_);
    }

    void FrankaHuskyWrapper::nle(VectorXd & nle_vec){
        if (ismobile_) nle_vec = robot_->nonLinearEffects(data_).tail(na_-2);
        else           nle_vec = robot_->nonLinearEffects(data_).tail(na_);
    }

    void FrankaHuskyWrapper::g(VectorXd & g_vec){
        //API:Vector of generalized gravity (dim model.nv).
        if (ismobile_) g_vec = data_.g.tail(na_-2);
        else           g_vec = data_.g.tail(na_);
    }

    void FrankaHuskyWrapper::g_joint7(VectorXd & g_vec){
        Vector3d g_global;
        g_global << 0.0, 0.0, -9.81;
        
        SE3 m_wMl;                
        robot_->framePosition(data_, robot_->model().getFrameId("panda_joint7"), m_wMl);
        m_wMl.translation() << 0.0, 0.0, 0.0; //transform only with rotation
        g_vec = m_wMl.actInv(g_global);
    }

    void FrankaHuskyWrapper::g_local_offset(VectorXd & g_vec){
        Vector3d g_global;
        g_global << 0.0, 0.0, -9.81;
        
        SE3 m_wMl;                
        m_wMl = robot_->position(data_, robot_->model().getJointId("panda_joint7")) * T_offset_;        
        m_wMl.translation() << 0.0, 0.0, 0.0; //transform only with rotation                
        g_vec = m_wMl.actInv(g_global);
    }

    void FrankaHuskyWrapper::JWorld(MatrixXd & Jo){
        Data::Matrix6x Jo2;        
        Jo2.resize(6, robot_->nv());
        robot_->jacobianWorld(data_, robot_->model().getJointId("panda_joint7"), Jo2);
        Jo = Jo2.bottomRightCorner(6, 7);
    }

    void FrankaHuskyWrapper::JLocal(MatrixXd & Jo){
        Data::Matrix6x Jo2;        
        Jo2.resize(6, robot_->nv());
        robot_->frameJacobianLocal(data_, robot_->model().getFrameId("panda_joint7"), Jo2);        
        Jo = Jo2.bottomRightCorner(6, 7);
    }

    void FrankaHuskyWrapper::JLocal_offset(MatrixXd & Jo){
        Data::Matrix6x Jo2;        
        Jo2.resize(6, robot_->nv());
        robot_->frameJacobianLocal(data_, robot_->model().getFrameId("panda_joint7"), Jo2);        
        Jo = Jo2.bottomRightCorner(6, 7);
        Jo = Adj_mat_ * Jo;
    }

    void FrankaHuskyWrapper::dJLocal(MatrixXd & dJo){
        Data::Matrix6x dJo2;        
        dJo2.resize(6, robot_->nv());
        robot_->frameJacobianTimeVariationLocal(data_, robot_->model().getFrameId("panda_joint7"), dJo2);        
        dJo = dJo2.bottomRightCorner(6, 7);
    }

    void FrankaHuskyWrapper::dJLocal_offset(MatrixXd & dJo){
        Data::Matrix6x dJo2;        
        dJo2.resize(6, robot_->nv());
        robot_->frameJacobianTimeVariationLocal(data_, robot_->model().getFrameId("panda_joint7"), dJo2);        
        dJo = dJo2.bottomRightCorner(6, 7);
        dJo = Adj_mat_ * dJo;
    }

    void FrankaHuskyWrapper::MxLocal_offset(MatrixXd & Mx){
        MatrixXd M, J, J_inverse, J_transpose_inverse;
        this->mass(M);
        this->JLocal_offset(J);
        J_inverse = J.completeOrthogonalDecomposition().pseudoInverse();
        J_transpose_inverse = J.transpose().completeOrthogonalDecomposition().pseudoInverse();        

        Mx = J_transpose_inverse * M * J_inverse;    
    }

    void FrankaHuskyWrapper::MxGlobal(MatrixXd & Mx){
        MatrixXd M, J, J_inverse, J_transpose_inverse;
        this->mass(M);
        this->JWorld(J);
        J_inverse = J.completeOrthogonalDecomposition().pseudoInverse();
        J_transpose_inverse = J.transpose().completeOrthogonalDecomposition().pseudoInverse();        

        Mx = J_transpose_inverse * M * J_inverse;    
    }    

    void FrankaHuskyWrapper::Fh_gain_matrix(MatrixXd & Fh, MatrixXd & Fh_local, bool isinteractiongain){
        if (isinteractiongain) {
            Fh = Fh_;   
            Fh_local = Fh_local_;     
        }
        else {
            Fh.setZero();
            Fh_local.setZero();                      
        }
    }

    void FrankaHuskyWrapper::ee_state(Vector3d & pos, Eigen::Quaterniond & quat){
        for (int i=0; i<3; i++)
            pos(i) = robot_->position(data_, robot_->model().getJointId("panda_joint7")).translation()(i);

        Quaternion<double> q(robot_->position(data_, robot_->model().getJointId("panda_joint7")).rotation());
        quat = q;
    }

    void FrankaHuskyWrapper::base_state(Vector3d & base){
        base(0) = robot_->getMobilePosition(data_, 5).translation()(0); //5 means state.q_ at panda joint 1 
        base(1) = robot_->getMobilePosition(data_, 5).translation()(1); //5 means state.q_ at panda joint 1 
        base(2) = atan2(-robot_->getMobilePosition(data_, 5).rotation()(0, 1),robot_->getMobilePosition(data_, 5).rotation()(0,0)); //5 means state.q_ at panda joint 1 
    }

    void FrankaHuskyWrapper::rotx(double & angle, Eigen::Matrix3d & rot){
        rot.row(0) << 1,           0,           0;
        rot.row(1) << 0,           cos(angle), -sin(angle);
        rot.row(2) << 0,           sin(angle),  cos(angle);
    }

    void FrankaHuskyWrapper::roty(double & angle, Eigen::Matrix3d & rot){
        rot.row(0) <<  cos(angle),           0,            sin(angle);
        rot.row(1) <<  0,                    1,            0 ;
        rot.row(2) << -sin(angle),           0,            cos(angle);
    }

    void FrankaHuskyWrapper::rotz(double & angle, Eigen::Matrix3d & rot){
        rot.row(0) << cos(angle), -sin(angle),  0;
        rot.row(1) << sin(angle),  cos(angle),  0;
        rot.row(2) << 0,                    0,  1;
    }    
    
    MatrixXd FrankaHuskyWrapper::skew_matrix(const VectorXd& vec){
        double v1 = vec(0);
        double v2 = vec(1);
        double v3 = vec(2);

        Eigen::Matrix3d CM;
        CM <<     0, -1*v3,    v2,
                 v3,     0, -1*v1,
              -1*v2,    v1,     0;

        return CM;
    }

    pinocchio::SE3 FrankaHuskyWrapper::vel_to_SE3(VectorXd vel, double dt){        
        Quaterniond angvel_quat(1, vel(3) * dt * 0.5, vel(4) * dt * 0.5, vel(5) * dt * 0.5);        
        Matrix3d Rotm = angvel_quat.normalized().toRotationMatrix();                
        
        pinocchio::SE3 T;
        T.translation() = vel.head(3) * dt;
        T.rotation() = Rotm;

        return T;
    }

    void FrankaHuskyWrapper::get_dt(double dt){
        dt_ = dt;
    }

    double FrankaHuskyWrapper::trajectory_length_in_time(){        
        return  traj_length_in_time_;           
    }

    double FrankaHuskyWrapper::noise_elimination(double x, double limit) {
        double y;
        if (abs(x) > limit) y = x;
        else y = 0.0;
        
        return y;
    }
}// namespace
