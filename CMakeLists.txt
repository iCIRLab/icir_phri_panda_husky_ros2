cmake_minimum_required(VERSION 3.8)
project(icir_phri_panda_husky)

# ROS1 원본과 같다. 1kHz HQP 루프라 최적화가 필요하고, Eigen 의 런타임 assert 도
# NDEBUG 로 꺼진다 (원본이 의존하던 동작).
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ---------------------------------------------------------------------------
# 파이썬 모듈
#
# launch가 import한다 (MJCF에서 장애물/액추에이터 조인트를 뽑는 helper).
# ROS1에서 전역 파라미터 서버를 거치던 값들을 launch 시점에 읽어 노드 파라미터로
# 넘기기 위한 것이라, 실행 파일이 아니라 import 가능한 모듈이어야 한다.
# ---------------------------------------------------------------------------
ament_python_install_package(${PROJECT_NAME})

# ---------------------------------------------------------------------------
# 파이썬 노드
#
# rosidl/ament_python 패키지가 아니라 ament_cmake이므로 스크립트를 lib/<pkg>에
# 직접 설치한다. 여기 들어가야 `ros2 run` / launch의 executable= 로 잡힌다.
# PROGRAMS는 실행 비트를 붙여 설치한다(FILES는 안 붙는다).
# ---------------------------------------------------------------------------
install(PROGRAMS
  python/ns1_real.py
  python/ns1_simulation.py
  python/obstacle_from_mujoco.py
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# 리소스
#
# launch/는 .launch.py로 전부 옮겼다. ROS1 XML 원본은
# ~/icir_phri_ws/src/icir_phri_panda_husky/launch/ 에 그대로 있다.
# ---------------------------------------------------------------------------
install(DIRECTORY
  launch
  config
  rviz
  rqt
  calibration
  DESTINATION share/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# C++ 컨트롤러
#
# 구성은 ROS1과 같다: panda_husky_hqp(HQP 코어)를 공유 라이브러리로 두고,
# 시뮬은 그 위의 독립 실행 노드(phri_simul), 실기는 컨트롤러 플러그인(phri_real).
# ---------------------------------------------------------------------------
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(icir_hqp_controller REQUIRED)
find_package(icir_object_estimation REQUIRED)
find_package(icir_path_planner_ros_interface REQUIRED)
find_package(icir_phri_msgs REQUIRED)
find_package(mujoco_ros2_control_msgs REQUIRED)

# pinocchio 헤더가 -Wpedantic 에서 시끄럽다. 원본에도 -Wall -Wextra 는 없었다.
add_compile_options(-Wno-pedantic)

# HQP 코어 (ROS1의 phri_hqp 라이브러리와 같은 이름/역할)
add_library(phri_hqp SHARED src/panda_husky_hqp.cpp)
target_include_directories(phri_hqp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(phri_hqp PUBLIC
  pinocchio::pinocchio
  icir_hqp_controller::icir_hqp_controller)
ament_target_dependencies(phri_hqp PUBLIC
  rclcpp std_msgs sensor_msgs geometry_msgs ament_index_cpp
  icir_path_planner_ros_interface)

# 시뮬 노드 (ROS1의 icir_phri_panda_husky_simulation)
add_executable(icir_phri_panda_husky_simulation src/phri_simul.cpp)
target_link_libraries(icir_phri_panda_husky_simulation
  phri_hqp
  icir_object_estimation::icir_object_estimation)
ament_target_dependencies(icir_phri_panda_husky_simulation
  rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs tf2_ros
  icir_phri_msgs mujoco_ros2_control_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS phri_hqp
  EXPORT export_phri_hqp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS icir_phri_panda_husky_simulation
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_phri_hqp HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# ===========================================================================
# 보류: 실기 컨트롤러 (phri_real)
#
# franka(클래식 Panda용 커뮤니티 ROS2 포팅본) / robotiq(ros2_robotiq_gripper) 이
# 워크스페이스에도 apt 에도 없어서 헤더조차 찾을 수 없다. 드라이버가 들어오면
# package.xml 의 동명 블록과 함께 되살린다.
#
# 되살릴 때 함께 처리해야 할 것 (계획서 4절 Phase 4):
#   - phri_real  -> controller_interface::ControllerInterface 플러그인 (R2)
#   - cfg/ekf_param.cfg (dynamic_reconfigure) -> 파라미터 콜백 (R4)
#   - 그리퍼 결합부 -> ros2_robotiq_gripper 타입 (R5)
#
#   add_library(${PROJECT_NAME} SHARED src/phri_real.cpp)
#   pluginlib_export_plugin_description_file(
#     controller_interface icir_phri_panda_husky_plugin.xml)
# ===========================================================================

ament_package()
