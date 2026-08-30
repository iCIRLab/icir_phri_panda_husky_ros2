# ns1_phri_simulation_w_viz.launch (ROS1) 의 ROS2 포팅본.
#
#   ros2 launch icir_phri_panda_husky ns1_phri_simulation_w_viz.launch.py
#   ros2 launch icir_phri_panda_husky ns1_phri_simulation_w_viz.launch.py mobile:=false
#   ros2 launch icir_phri_panda_husky ns1_phri_simulation_w_viz.launch.py viz:=true
#
# ROS1과 달라진 점
# ----------------
# 1. mujoco: 커스텀 브릿지 `mujoco_ros` 노드 -> `mujoco_ros2_control`의
#    ros2_control_node (계획서 R3). 씬은 그대로 쓰고, ros2_control <joint> 블록을
#    URDF에 문자열로 주입한다. 조인트 이름은 URDF가 아니라 MJCF 액추에이터 기준이라
#    씬에서 직접 읽는다(icir_phri_panda_husky.mjcf.actuator_joints).
#
# 2. 파라미터: ROS1은 <param>을 전역 파라미터 서버에 올려두고 컨트롤러가
#    getParam으로 읽었다. ROS2엔 전역 파라미터 서버가 없어서 컨트롤러 노드의
#    파라미터로 직접 넘긴다. 아래 CONTROLLER_PARAMS가 그 목록이고, ROS1이 올리던
#    이름/값을 그대로 유지했다 - C++ 포팅 때 이게 계약서 역할을 한다.
#
# 3. 컨트롤러 노드: ROS1과 같은 독립 실행 노드다 (ros2_control 플러그인이 아니다).
#    MujocoRosInterfacePlugin 과 ROS1 그대로의 토픽 이름/JointSet 인덱스 규약으로
#    이야기한다. controller:=false 로 끄면 mujoco + robot_state_publisher 만 뜬다.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from icir_phri_panda_husky.mjcf import actuator_joints

NS = 'ns1'


# mobile 여부로 갈리는 구성. ROS1 launch의 <group if="$(arg mobile)"> /
# <group unless=...> 두 갈래를 그대로 옮긴 것.
CONFIGS = {
    True: dict(
        desc_pkg='icir_panda_husky_description',
        scene='mujoco_simulation_phri.xml',
        urdf='husky_panda_hand.urdf',            # pinocchio용 (컨트롤러가 읽는다)
        rviz_urdf='husky_panda_hand_rviz_ns1.urdf',  # tf/robot_state_publisher용
        perspective='icir_phri_panda_husky_sim.perspective',
    ),
    False: dict(
        desc_pkg='icir_panda_description',
        scene='mujoco_simulation_phri.xml',
        urdf='panda_arm_hand_l.urdf',
        rviz_urdf='panda_arm_hand_l_rviz.urdf',
        perspective='icir_phri_panda_sim.perspective',
    ),
}


def _ros2_control_block(scene_path, joints, headless):
    """URDF 끝에 붙일 <ros2_control> 블록.

    effort 지령 + position/velocity/effort 상태. ROS1 컨트롤러가 mujoco_ros_msgs::
    JointSet으로 토크를 쏘던 것에 대응한다.
    """
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
    mobile = LaunchConfiguration('mobile').perform(context).lower() == 'true'
    viz = LaunchConfiguration('viz').perform(context).lower() == 'true'
    headless = LaunchConfiguration('headless').perform(context).lower()
    controller = LaunchConfiguration('controller').perform(context).lower() == 'true'

    cfg = CONFIGS[mobile]
    desc = get_package_share_directory(cfg['desc_pkg'])
    scene_path = os.path.join(desc, 'robots', cfg['scene'])
    rviz_urdf_path = os.path.join(desc, 'robots', cfg['rviz_urdf'])

    # robot_state_publisher가 tf를 뿌리는 데 쓰는 모델. ROS1도 여기엔 _rviz 변형본을
    # 썼다 (컨트롤러가 pinocchio로 읽는 모델과 다르다).
    with open(rviz_urdf_path) as f:
        robot_desc = f.read()

    joints = actuator_joints(scene_path)
    robot_desc_with_control = robot_desc.replace(
        '</robot>', _ros2_control_block(scene_path, joints, headless) + '</robot>')

    # ROS1이 전역 파라미터 서버에 올리던 값들. 이름/값 그대로다.
    controller_params = {
        'robot_group': NS,
        'robotiq_gripper': False,
        'mobile': mobile,
        'issimulation': True,
        'iscalibration': False,
        # pinocchio가 읽을 모델. ROS1처럼 경로와 파일명을 따로 준다.
        'robot_urdf_path': desc,
        'robot_urdf': '/robots/' + cfg['urdf'],
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
                                         'rqt', cfg['perspective'])]))

    if controller:
        nodes.append(
            Node(package='icir_phri_panda_husky',
                 executable='icir_phri_panda_husky_simulation',
                 namespace=NS, output='screen',
                 arguments=['-keyboard'],
                 parameters=[controller_params]))
    else:
        nodes.append(LogInfo(msg=(
            '[ns1_phri_simulation] controller:=false 라 컨트롤러를 띄우지 않습니다. '
            'mujoco + robot_state_publisher만 뜹니다.')))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mobile', default_value='true',
                              description='true면 husky+panda, false면 panda 단독'),
        DeclareLaunchArgument('viz', default_value='false',
                              description='true면 rqt_gui도 띄운다'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='true면 MuJoCo 창 없이 구동(스모크 테스트)'),
        DeclareLaunchArgument('controller', default_value='true',
                              description='false면 mujoco+robot_state_publisher만 띄운다'),
        OpaqueFunction(function=_setup),
    ])
