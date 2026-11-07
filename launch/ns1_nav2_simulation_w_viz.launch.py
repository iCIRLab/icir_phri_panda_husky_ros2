import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from icir_navigation.occupancy_map import cache_dir, write_box_map
from icir_phri_panda_husky.mjcf import actuator_joints, parse_obstacles

NS = 'ns1'

DESC_PKG = 'icir_panda_husky_description'
SCENE = 'mujoco_simulation_nav2.xml'
URDF = 'husky_panda_hand.urdf'
RVIZ_URDF = 'husky_panda_hand_rviz_ns1.urdf'
PERSPECTIVE = 'icir_phri_panda_husky_sim.perspective'

OBSTACLE_PREFIX = 'obs'

LIDAR_SITE = 'lidar_rf'
LIDAR_FRAME = NS + '_lidar_link'
LIDAR_BASE_FRAME = NS + '_rviz_base_link'
LIDAR_RANGE = (0.05, 15.0)
LIDAR_RATE = 10.0


def _lidar_geometry(robot_xml):
    """robot MJCF 에서 lidar 레이 개수/간격/장착 위치를 읽는다.

    husky_panda_hand.xml 의 <frame pos=...><replicate count=N euler="0 d 0"><site name=lidar_rf>
    구조를 전제한다 (첫 레이가 -pi 를 보고 d 씩 반시계로 돈다).
    """
    root = ET.parse(robot_xml).getroot()
    for frame in root.iter('frame'):
        rep = frame.find('replicate')
        if rep is None or rep.find("site[@name='%s']" % LIDAR_SITE) is None:
            continue
        count = int(rep.attrib['count'])
        inc = float(rep.attrib['euler'].split()[1])   # compiler angle="radian"
        pos = [float(v) for v in frame.attrib.get('pos', '0 0 0').split()]
        return count, inc, pos
    raise RuntimeError("no <frame><replicate><site name='%s'> in %s" % (LIDAR_SITE, robot_xml))


def _ros2_control_block(scene_path, joints, headless, lidar):
    count, inc, _ = lidar
    min_angle = -math.pi
    max_angle = min_angle + (count - 1) * inc + 1e-6 * inc
    assert int((max_angle - min_angle) / inc) + 1 == count

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
    <param name="lidar_publish_rate">{LIDAR_RATE}</param>
  </hardware>
{joint_xml}
  <sensor name="lidar">
    <param name="frame_name">{LIDAR_FRAME}</param>
    <param name="angle_increment">{inc!r}</param>
    <param name="min_angle">{min_angle!r}</param>
    <param name="max_angle">{max_angle!r}</param>
    <param name="range_min">{LIDAR_RANGE[0]}</param>
    <param name="range_max">{LIDAR_RANGE[1]}</param>
    <param name="laserscan_topic">/{NS}/scan</param>
  </sensor>
</ros2_control>
'''


def _setup(context, *args, **kwargs):
    viz = LaunchConfiguration('viz').perform(context).lower() == 'true'
    rviz = LaunchConfiguration('rviz').perform(context).lower()
    headless = LaunchConfiguration('headless').perform(context).lower()
    controller = LaunchConfiguration('controller').perform(context).lower() == 'true'
    nav2 = LaunchConfiguration('nav2').perform(context).lower() == 'true'
    slam = LaunchConfiguration('slam').perform(context).lower()
    mapfile = LaunchConfiguration('map').perform(context)

    desc = get_package_share_directory(DESC_PKG)
    scene_path = os.path.join(desc, 'robots', SCENE)
    lidar = _lidar_geometry(os.path.join(desc, 'robots', 'husky_panda_hand.xml'))

    with open(os.path.join(desc, 'robots', RVIZ_URDF)) as f:
        robot_desc = f.read()

    joints = actuator_joints(scene_path)
    robot_desc_with_control = robot_desc.replace(
        '</robot>', _ros2_control_block(scene_path, joints, headless, lidar) + '</robot>')

    controller_params = {
        'robot_group': NS,
        'robotiq_gripper': False,
        'mobile': True,
        'issimulation': True,
        'iscalibration': False,
        'robot_urdf_path': desc,
        'robot_urdf': '/robots/' + URDF,
        'use_sim_time': True,
        'publish_odom': True,
    }

    lx, ly, lz = lidar[2]
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
                 {'use_sim_time': True},
             ]),

        Node(package='tf2_ros', executable='static_transform_publisher',
             name='lidar_static_tf', namespace=NS, output='screen',
             arguments=['--x', str(lx), '--y', str(ly), '--z', str(lz),
                        '--frame-id', LIDAR_BASE_FRAME, '--child-frame-id', LIDAR_FRAME],
             parameters=[{'use_sim_time': True}]),
    ]

    if viz:
        nodes.append(
            Node(package='rqt_gui', executable='rqt_gui', namespace=NS, output='screen',
                 arguments=['--perspective-file',
                            os.path.join(get_package_share_directory('icir_phri_panda_husky'),
                                         'rqt', PERSPECTIVE)]))

    if controller:
        nodes.append(
            Node(package='icir_phri_panda_husky',
                 executable='icir_phri_panda_husky_simulation',
                 namespace=NS, output='screen',
                 arguments=['-keyboard'],
                 parameters=[controller_params]))

    if nav2:
        if slam != 'true' and not mapfile:
            mapfile = write_box_map(parse_obstacles(scene_path, OBSTACLE_PREFIX),
                                    os.path.join(cache_dir(), os.path.splitext(SCENE)[0]))
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('icir_navigation'), 'launch',
                    'navigation.launch.py')),
                launch_arguments={
                    'namespace': NS,
                    'map': mapfile,
                    'slam': slam,
                    'rviz': rviz,
                    'initial_pose': '0,0,0',
                }.items()))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Launch RViz'),
        DeclareLaunchArgument('viz', default_value='false',
                              description='Launch rqt_gui'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run MuJoCo without a window'),
        DeclareLaunchArgument('controller', default_value='true',
                              description='Launch the controller'),
        DeclareLaunchArgument('nav2', default_value='true',
                              description='Launch Nav2'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='Use SLAM instead of static map + AMCL'),
        DeclareLaunchArgument('map', default_value='',
                              description='Map yaml (empty: build from scene)'),
        OpaqueFunction(function=_setup),
    ])
