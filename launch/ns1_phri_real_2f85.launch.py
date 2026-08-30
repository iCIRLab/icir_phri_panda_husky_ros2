# ns1_phri_real_2f85.launch (ROS1) 의 ROS2 포팅본. 실기(클래식 Panda + Husky + Robotiq 2F-85).
#
#   ros2 launch icir_phri_panda_husky ns1_phri_real_2f85.launch.py
#   ros2 launch icir_phri_panda_husky ns1_phri_real_2f85.launch.py mobile:=false
#
# ⚠ 현재 실기 경로는 뜨지 않는다. 지금 뜨는 건 robot_state_publisher(tf) 뿐이다.
#    franka 드라이버와 컨트롤러가 아직 없기 때문이고, 아래 두 인자로 켠다:
#      franka:=true     franka 브링업 (계획서 R1 - 커뮤니티 Panda ROS2 포팅본 필요.
#                       공식 franka_ros2는 FR3 전용이라 클래식 Panda엔 못 쓴다)
#      controller:=true 컨트롤러 스폰 (계획서 R2 - phri_real을
#                       controller_interface::ControllerInterface로 재작성 필요)
#
# ROS1과 달라진 점
# ----------------
# 1. 파라미터: ROS1은 <param>을 전역 파라미터 서버에 올려 컨트롤러가 getParam으로
#    읽었다. ROS2엔 전역 파라미터 서버가 없어서 컨트롤러 노드 파라미터로 직접
#    넘긴다. CONTROLLER_PARAMS가 그 목록이며 ROS1의 이름/값을 그대로 유지했다 -
#    C++ 포팅 때 이게 계약서 역할을 한다.
#
# 2. 컨트롤러 스폰: <node pkg="controller_manager" type="spawner"> ->
#    ros2_control의 `spawner` 실행 파일. 컨트롤러 설정(config/*.yaml)은 ROS1처럼
#    통째로 파라미터 서버에 올리는 게 아니라 controller_manager 노드에 넘긴다.
#
# 3. 그리퍼: 커스텀 robotiq 패키지 -> 공식 ros2_robotiq_gripper (계획서 R5).
#    코어의 그리퍼 클라이언트 재작성이 따라와야 한다.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

NS = 'ns1'

# mobile 여부로 갈리는 구성. ROS1 launch의 <group if/unless="$(arg mobile)"> 그대로.
CONFIGS = {
    True: dict(
        desc_pkg='icir_panda_husky_description',
        urdf='husky_panda_2f_85.urdf',                 # pinocchio용
        gui_urdf='husky_panda_2f_85_rviz_ns1.urdf',    # gui 조인트 이름 / tf용
    ),
    False: dict(
        desc_pkg='icir_panda_description',
        urdf='panda_arm_2f_85.urdf',
        gui_urdf='panda_arm_2f_85_rviz.urdf',
    ),
}


def _setup(context, *args, **kwargs):
    mobile = LaunchConfiguration('mobile').perform(context).lower() == 'true'
    calibration = LaunchConfiguration('calibration').perform(context).lower() == 'true'
    load_gripper = LaunchConfiguration('load_gripper').perform(context).lower() == 'true'
    robot_ip = LaunchConfiguration('robot_ip').perform(context)
    franka = LaunchConfiguration('franka').perform(context).lower() == 'true'
    controller = LaunchConfiguration('controller').perform(context).lower() == 'true'

    cfg = CONFIGS[mobile]
    desc = get_package_share_directory(cfg['desc_pkg'])
    gui_urdf_path = os.path.join(desc, 'robots', cfg['gui_urdf'])

    # mobile:=false 갈래가 참조하는 panda_arm_2f_85*.urdf 는 ROS1 원본 description
    # 에도 없다(끊어진 참조). 스택트레이스 대신 뭐가 없는지 말해주고 끝낸다.
    if not os.path.isfile(gui_urdf_path):
        return [LogInfo(msg=(
            f"[ns1_phri_real] '{gui_urdf_path}' 가 없어서 띄울 수 없습니다. "
            f"{cfg['desc_pkg']} 에 {cfg['gui_urdf']} / {cfg['urdf']} 가 들어와야 합니다 "
            '(ROS1 원본 description에도 없던 파일입니다).'))]

    with open(gui_urdf_path) as f:
        robot_desc = f.read()

    # ROS1이 전역 파라미터 서버에 올리던 값들. 이름/값 그대로다.
    controller_params = {
        'robot_group': NS,
        'load_gripper': False,      # ROS1도 arg와 무관하게 false 고정이었다
        'robotiq_gripper': True,
        'mobile': mobile,
        'issimulation': False,
        'iscalibration': calibration,
        'robot_urdf_path': desc,
        'robot_urdf': '/robots/' + cfg['urdf'],
        'gui_urdf_path': desc,
        'gui_urdf_name': '/robots/' + cfg['gui_urdf'],
    }

    nodes = [
        # tf. ROS1처럼 joint_states를 real_robot/joint_states 에서 받는다.
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             namespace=NS, output='screen', respawn=True,
             parameters=[{'robot_description': robot_desc}],
             remappings=[('joint_states', 'real_robot/joint_states')]),
    ]

    if franka:
        # 계획서 R1에서 포팅본을 확정하면 여기에 해당 패키지의 브링업을 include한다.
        # robot_ip / load_gripper 를 그쪽 인자로 넘기면 된다.
        nodes.append(LogInfo(msg=(
            '[ns1_phri_real] franka:=true 이지만 Panda ROS2 드라이버가 아직 없습니다. '
            f'(robot_ip={robot_ip}, load_gripper={load_gripper}) 계획서 R1 참조.')))

    if controller:
        nodes.append(
            Node(package='controller_manager', executable='spawner',
                 namespace=NS, output='screen',
                 arguments=['phri_franka_husky_controller',
                            '--param-file',
                            os.path.join(get_package_share_directory('icir_phri_panda_husky'),
                                         'config', 'icir_phri_panda_husky_controllers.yaml')],
                 parameters=[controller_params]))
    else:
        nodes.append(LogInfo(msg=(
            '[ns1_phri_real] 컨트롤러는 아직 ROS2 포팅 전이라 스폰하지 않습니다. '
            'robot_state_publisher(tf)만 뜹니다. 포팅 후 controller:=true.')))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.10.52',
                              description='Panda 제어박스 IP'),
        DeclareLaunchArgument('mobile', default_value='true',
                              description='true면 husky+panda, false면 panda 단독'),
        DeclareLaunchArgument('calibration', default_value='false',
                              description='iscalibration 파라미터로 전달된다'),
        DeclareLaunchArgument('load_gripper', default_value='false',
                              description='franka 그리퍼(=panda hand) 사용 여부'),
        DeclareLaunchArgument('franka', default_value='false',
                              description='true면 franka 브링업 (아직 드라이버 없음, R1)'),
        DeclareLaunchArgument('controller', default_value='false',
                              description='true면 컨트롤러 스폰 (아직 포팅 전, R2)'),
        OpaqueFunction(function=_setup),
    ])
