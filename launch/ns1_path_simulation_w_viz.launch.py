# ns1_path_simulation_w_viz.launch (ROS1) 의 ROS2 포팅본.
#
#   ros2 launch icir_phri_panda_husky ns1_path_simulation_w_viz.launch.py
#   ros2 launch icir_phri_panda_husky ns1_path_simulation_w_viz.launch.py viz:=true
#
# ns1_phri_simulation_w_viz 와 같은 구성인데 path-planner 씬(장애물이 있는
# mujoco_simulation_path.xml)을 쓰고 장애물 목록을 넘긴다는 점이 다르다.
#
# ROS1과 달라진 점
# ----------------
# 1. 장애물: ROS1은 obstacle_from_mujoco.py 노드가 씬을 파싱해 rospy.set_param()으로
#    전역 파라미터 서버에 올리면 컨트롤러가 getParam으로 읽어갔다. ROS2엔 전역
#    파라미터 서버가 없어서, launch 시점에 파싱해 컨트롤러 노드 파라미터로 직접
#    넘긴다. MJCF는 실행 중 안 바뀌는 정적 파일이라 노드를 띄울 이유가 없다.
#
#    ⚠ ROS2 파라미터는 1-D 배열만 되고 중첩 배열이 안 된다. ROS1이 올리던
#    [[cx,cy,hx,hy], ...] 를 길이 4N으로 평탄화해 넘기므로, 받는 쪽
#    (panda_husky_hqp.cpp의 setMujocoObstacles)은 4개씩 끊어 읽어야 한다.
#
# 2. mujoco: 커스텀 브릿지 `mujoco_ros` -> `mujoco_ros2_control` (계획서 R3).
#
# 3. path-planner 서버: 컨트롤러와 같은 네임스페이스에 함께 띄운다. ROS1은 서비스가
#    노드 private 이름(/ns1/icir_path_planner_ros_interface_server/plan_mobile_path)이
#    었는데, ROS2 포팅본은 상대 이름이라 /ns1/plan_mobile_path 가 된다.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from icir_phri_panda_husky.mjcf import actuator_joints, parse_obstacles_flat

NS = 'ns1'

DESC_PKG = 'icir_panda_husky_description'
SCENE = 'mujoco_simulation_path.xml'
URDF = 'husky_panda_hand.urdf'                 # pinocchio용 (컨트롤러가 읽는다)
RVIZ_URDF = 'husky_panda_hand_rviz_ns1.urdf'   # tf/robot_state_publisher용
PERSPECTIVE = 'icir_phri_panda_husky_sim.perspective'

# 컨트롤러가 planner의 footprint inflation 위에 추가로 두는 여유 [m]. ROS1 값 그대로.
OBSTACLE_MARGIN = 0.15
# 장애물로 볼 geom 이름 접두어. ROS1의 obstacle_prefix 파라미터 기본값 그대로.
OBSTACLE_PREFIX = 'obs'


def _ros2_control_block(scene_path, joints, headless):
    joint_xml = '\n'.join(
        f'''    <joint name="{j}">
      <command_interface name="effort"/>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>''' for j in joints)

    return f'''
<ros2_control name="MujocoSystem" type="system">
  <hardware>
    <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
    <param name="mujoco_model">{scene_path}</param>
    <param name="headless">{headless}</param>
  </hardware>
{joint_xml}
</ros2_control>
'''


def _setup(context, *args, **kwargs):
    viz = LaunchConfiguration('viz').perform(context).lower() == 'true'
    headless = LaunchConfiguration('headless').perform(context).lower()
    controller = LaunchConfiguration('controller').perform(context).lower() == 'true'

    desc = get_package_share_directory(DESC_PKG)
    scene_path = os.path.join(desc, 'robots', SCENE)

    with open(os.path.join(desc, 'robots', RVIZ_URDF)) as f:
        robot_desc = f.read()

    joints = actuator_joints(scene_path)
    robot_desc_with_control = robot_desc.replace(
        '</robot>', _ros2_control_block(scene_path, joints, headless) + '</robot>')

    # ROS1이 전역 파라미터 서버에 올리던 값들. 이름/값 그대로다.
    controller_params = {
        'robot_group': NS,
        'robotiq_gripper': False,
        'mobile': True,
        'issimulation': True,
        'iscalibration': False,
        'robot_urdf_path': desc,
        'robot_urdf': '/robots/' + URDF,
        'obstacle_margin': OBSTACLE_MARGIN,
        'obstacles': parse_obstacles_flat(scene_path, OBSTACLE_PREFIX),
    }

    nodes = [
        Node(package='mujoco_ros2_control', executable='ros2_control_node',
             namespace=NS, output='screen',
             parameters=[
                 {'use_sim_time': True},
                 {'robot_description': robot_desc_with_control},
                 {'update_rate': 1000},
                 {'mujoco_plugins.mujoco_ros_interface.type':
                  'mujoco_ros2_control_plugins/MujocoRosInterfacePlugin'},
             ],
             remappings=[('~/robot_description', 'robot_description')]),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=NS, output='screen',
             parameters=[
                 {'robot_description': robot_desc},
                 {'publish_frequency': 100.0},
                 {'ignore_timestamp': True},
             ]),
    ]

    if viz:
        nodes.append(
            Node(package='rqt_gui', executable='rqt_gui', namespace=NS, output='screen',
                 arguments=['--perspective-file',
                            os.path.join(get_package_share_directory('icir_phri_panda_husky'),
                                         'rqt', PERSPECTIVE)]))

    if controller:
        # 컨트롤러가 mode 104/105 에서 부르는 경로계획 서버. 같은 네임스페이스여야
        # 상대 이름 'plan_mobile_path' 가 맞물린다.
        planner_share = get_package_share_directory('icir_path_planner_ros_interface')
        nodes.append(
            Node(package='icir_path_planner_ros_interface',
                 executable='icir_path_planner_ros_interface_server',
                 name='icir_path_planner_ros_interface_server',
                 namespace=NS, output='screen',
                 parameters=[{
                     'save_path': os.path.join(planner_share, 'result', 'mobile.json'),
                     'robot_planning_model': os.path.join(
                         get_package_share_directory('icir_path_planner'),
                         'robot_model', 'car.svg'),
                 }]))
        nodes.append(
            Node(package='icir_phri_panda_husky',
                 executable='icir_phri_panda_husky_simulation',
                 namespace=NS, output='screen',
                 arguments=['-keyboard'],
                 parameters=[controller_params]))
    else:
        nodes.append(LogInfo(msg=(
            '[ns1_path_simulation] controller:=false 라 컨트롤러와 path-planner 서버를 '
            '띄우지 않습니다. mujoco + robot_state_publisher만 뜹니다. '
            '넘겨줄 장애물 파라미터: %r' % (controller_params['obstacles'],))))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('viz', default_value='false',
                              description='true면 rqt_gui도 띄운다'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true면 MuJoCo 창 없이 구동(스모크 테스트)'),
        DeclareLaunchArgument('controller', default_value='true',
                              description='false면 mujoco+robot_state_publisher만 띄운다'),
        OpaqueFunction(function=_setup),
    ])
