"""MJCF(MuJoCo XML) 씬에서 launch가 필요로 하는 값을 뽑아내는 helper.

ROS1에서는 이런 값들이 런타임에 전역 파라미터 서버를 거쳐 오갔다. ROS2에는 전역
파라미터 서버가 없고, MJCF는 실행 중 바뀌지 않는 정적 파일이므로 launch 시점에
읽어서 노드 파라미터로 바로 넘긴다.

여기 두 가지가 들어 있다:
  parse_obstacles / parse_obstacles_flat : path-planner 장애물 목록
  actuator_joints                        : ros2_control에 등록할 조인트 이름

둘 다 launch에서 import해 쓴다. `ros2 run`으로 도는 노드가 아니다.
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

__all__ = ['parse_obstacles', 'parse_obstacles_flat', 'actuator_joints']


def _warn(msg):
    print("[mjcf] " + msg, file=sys.stderr)


def _info(msg):
    print("[mjcf] " + msg)


def _floats(text):
    return [float(v) for v in text.replace(',', ' ').split()]


def _load_root(model_file):
    """Load the XML root, tolerating MuJoCo's sloppier XML.

    MJCF 파일들은 엄밀한 XML이 아닌 경우가 있는데 MuJoCo 파서는 넘어간다. 우리가
    쓰는 description에서 실제로 나온 두 가지를 여기서 흡수한다. 원본 파일을 고치는
    대신 여기서 처리하는 이유는, description을 ROS1 원본과 1:1로 유지하기로 했기
    때문이다.
    """
    try:
        return ET.parse(model_file).getroot()
    except ET.ParseError:
        import re
        with open(model_file) as f:
            text = f.read()
        # (1) 따옴표 없는 숫자 속성:  force = 0.1  ->  force="0.1"
        text = re.sub(r'=\s*([-+0-9.eE]+)(?=[\s/>])', r'="\1"', text)
        # (2) 속성 사이 공백 누락:  mesh="link6"rgba=".95 …"  ->  mesh="link6" rgba=".95 …"
        #     (panda_arm_hand_l.xml 42행에 실재하는 오타. 뒤가 `이름=` 꼴일 때만
        #      끼워넣으므로 본문 텍스트를 건드릴 위험은 없다.)
        text = re.sub(r'"(?=[A-Za-z_][A-Za-z0-9_.:-]*\s*=)', '" ', text)
        _warn("'%s' is not strict XML; parsed leniently" % model_file)
        return ET.fromstring(text)


# ---------------------------------------------------------------------------
# 장애물
#
# Convention (name method): only <geom type="box"> whose `name` starts with
# `prefix` (default "obs") is a base obstacle. This excludes the manipulated
# `object`, the robot, etc. The named collision geom is used; unnamed viz
# duplicates are ignored.
#
# Each obstacle is [cx, cy, half_x, half_y] (world frame, meters) -- the format
# the controller reads. The controller adds its own `obstacle_margin`, so no
# margin is baked in here.
# ---------------------------------------------------------------------------
def _yaw_from_quat(q):
    # MuJoCo quat order is (w, x, y, z)
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _geom_yaw(geom, angle_in_degree):
    """Best-effort planar yaw (rad) of a geom from its orientation attributes."""
    if 'quat' in geom.attrib:
        vals = _floats(geom.attrib['quat'])
        if len(vals) == 4:
            return _yaw_from_quat(vals)
    if 'euler' in geom.attrib:
        vals = _floats(geom.attrib['euler'])
        if len(vals) == 3:
            yaw = vals[2]
            return math.radians(yaw) if angle_in_degree else yaw
    if any(k in geom.attrib for k in ('axisangle', 'xyaxes', 'zaxis')):
        _warn("geom '%s' uses axisangle/xyaxes/zaxis; yaw ignored (treated as axis-aligned)"
              % geom.attrib.get('name', '?'))
    return 0.0


def parse_obstacles(model_file, prefix='obs'):
    """[[cx, cy, half_x, half_y], ...] (world frame, meters)."""
    root = _load_root(model_file)

    # MuJoCo angle unit: <compiler angle="degree|radian"/> (default: degree)
    angle_in_degree = True
    comp = root.find('compiler')
    if comp is not None and comp.attrib.get('angle', 'degree').lower().startswith('rad'):
        angle_in_degree = False

    obstacles = []
    for geom in root.iter('geom'):
        name = geom.attrib.get('name', '')
        if not name.startswith(prefix):
            continue
        if geom.attrib.get('type', '') != 'box':
            _warn("'%s' is not a box; skipped" % name)
            continue
        if 'pos' not in geom.attrib or 'size' not in geom.attrib:
            _warn("'%s' missing pos/size; skipped" % name)
            continue

        pos = _floats(geom.attrib['pos'])
        size = _floats(geom.attrib['size'])
        cx, cy = pos[0], pos[1]
        hx, hy = size[0], size[1]

        yaw = _geom_yaw(geom, angle_in_degree)
        if abs(yaw) > 1e-6:
            # axis-aligned bounding box of the rotated rectangle
            c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
            hx, hy = hx * c + hy * s, hx * s + hy * c

        obstacles.append([cx, cy, hx, hy])
        _info("%s -> [cx=%.3f cy=%.3f hx=%.3f hy=%.3f]" % (name, cx, cy, hx, hy))
    return obstacles


def parse_obstacles_flat(model_file, prefix='obs'):
    """ROS2 파라미터로 넘기기 위한 평탄화 버전 (길이 4N의 double 배열).

    ⚠ ROS2 파라미터는 1-D 배열만 되고 중첩 배열이 안 된다. ROS1이 파라미터 서버에
    올리던 [[cx,cy,hx,hy], ...] 를 그대로 넘길 수 없어서 평탄화한다. 받는 쪽
    (panda_husky_hqp.cpp의 setMujocoObstacles)은 4개씩 끊어 읽어야 한다.

    파싱이 실패해도 예외를 밖으로 내지 않는다 - launch 전체가 죽는 것보다 장애물
    없이 planning하는 편이 낫다는 ROS1판 동작을 유지한다.
    """
    try:
        obstacles = parse_obstacles(model_file, prefix)
    except Exception as e:  # noqa: BLE001 - report and return empty rather than kill the launch
        _warn("failed to parse '%s': %s" % (model_file, e))
        return []

    flat = [float(v) for obs in obstacles for v in obs]
    _info("%d obstacle(s) -> %d-element flat param" % (len(obstacles), len(flat)))
    return flat


# ---------------------------------------------------------------------------
# 액추에이터 조인트
# ---------------------------------------------------------------------------
def actuator_joints(scene_file):
    """씬이 구동하는 조인트 이름을 MJCF 순서 그대로 돌려준다.

    ros2_control의 <joint> 블록은 URDF가 아니라 MJCF 액추에이터 기준이어야 한다 -
    mujoco_ros2_control이 이름으로 액추에이터를 찾기 때문이다. 구성마다 이름이
    달라서(예: husky+panda는 'panda_joint1', panda 단독은 'l_panda_joint1')
    launch에 하드코딩하면 씬을 바꿀 때 조용히 어긋난다. 그래서 씬에서 직접 읽는다.

    씬 파일이 <include>로 로봇 본체를 끌어오므로 include를 따라 들어간다.
    """
    joints = []
    seen = set()
    visited = set()

    def walk(path):
        path = os.path.realpath(path)
        if path in visited:  # 순환 include 방지
            return
        visited.add(path)

        root = _load_root(path)
        base = os.path.dirname(path)

        for inc in root.iter('include'):
            target = inc.attrib.get('file')
            if target:
                walk(os.path.join(base, target))

        for act in root.iter('actuator'):
            for motor in act:
                joint = motor.attrib.get('joint')
                if joint and joint not in seen:
                    seen.add(joint)
                    joints.append(joint)

    walk(scene_file)
    _info("%s -> %d actuator joint(s)" % (os.path.basename(scene_file), len(joints)))
    return joints
