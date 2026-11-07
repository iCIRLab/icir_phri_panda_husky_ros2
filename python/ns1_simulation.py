#! /usr/bin/python3

import time

import rclpy
from std_msgs.msg import String
import cmd


class bcolors:
    OKBLUE = '\033[94m'
    ENDC = '\033[0m'


class ControlSuiteShell(cmd.Cmd):
    intro = bcolors.OKBLUE + "Welcome to the control suite shell.\nType help or ? to list commands.\n" + bcolors.ENDC
    prompt = "(csuite) "

    def __init__(self):
        cmd.Cmd.__init__(self)
        rclpy.init()
        self.node = rclpy.create_node('phri_simulation_controlsuite')
        # depth=1은 ROS1의 queue_size=1을 그대로 옮긴 것. 키 입력은 사람 손 속도라
        # 쌓일 일이 없고, 밀릴 경우 최신 명령만 살아남는 편이 맞다.
        self.key_pub = self.node.create_publisher(
            String, '/ns1/mujoco_ros/mujoco_ros_interface/remote_key', 1)
        # 이 노드는 발행만 하고 spin 하지 않는다. 발행 전 대기는 _pub() 참조.
        self._settled = False

    def _pub(self, key):
        # ROS2 는 discovery 가 비동기라, 구독자(컨트롤러)가 붙기 전에 publish 하면
        # 키가 조용히 유실된다. 고정 sleep 은 컨트롤러를 나중에 띄우거나 재시작하면
        # 다시 어긋나므로, 매칭될 때까지 기다렸다가 보낸다.
        deadline = time.time() + 5.0
        while self.key_pub.get_subscription_count() == 0 and time.time() < deadline:
            time.sleep(0.05)
        if self.key_pub.get_subscription_count() == 0:
            print("no subscriber on %s -- is the controller running?"
                  % self.key_pub.topic_name)
            return
        # 매칭이 잡힌 직후에는 writer 가 아직 자리를 못 잡아 첫 샘플이 버려진다.
        # 최초 1회만 잠깐 재운다. (환경 따라 더 필요하면 이 값을 올린다)
        if not self._settled:
            time.sleep(0.5)
            self._settled = True
        self.key_pub.publish(String(data=key))

    # ------------- basic motion -------------------------------------------
    def do_gravity(self, arg):
        'Gravity compensation mode [g]'
        self._pub('g')
        print("Gravity mode")

    def do_home(self, arg):
        'Go to home position [h]'
        self._pub('h')
        print("home position")

    def do_hold(self, arg):
        'Stop and maintain current position [f]'
        self._pub('f')
        print("stop with maintaining current position")

    # ------------- estimation motion --------------------------------------
    def do_est_arm(self, arg):
        'Identification motion w/o mobile [w]'
        self._pub('w')
        print("identification motion w/o mobile motion")

    def do_est_mobile(self, arg):
        'Identification motion w/ mobile [e]'
        self._pub('e')
        print("identification motion w/ mobile motion")

    # ------------- control mode -------------------------------------------
    def do_admittance(self, arg):
        'Admittance control [c]'
        self._pub('c')
        print("admittance control")

    def do_impedance(self, arg):
        'Impedance control [v]'
        self._pub('v')
        print("impedance control")

    def do_impedance_k(self, arg):
        'Impedance control with nonzero K, maintain current pose [n]'
        self._pub('n')
        print("impedance control with nonzero K")

    # ------------- arm EE jog ---------------------------------------------
    def do_ee_pz(self, arg):
        'Move EE +0.1m along z [i]'
        self._pub('i')
        print("move ee +0.1 z")

    def do_ee_nz(self, arg):
        'Move EE -0.1m along z [k]'
        self._pub('k')
        print("move ee -0.1 z")

    def do_ee_px(self, arg):
        'Move EE +0.1m along x [l]'
        self._pub('l')
        print("move ee +0.1 x")

    def do_ee_nx(self, arg):
        'Move EE -0.1m along x [j]'
        self._pub('j')
        print("move ee -0.1 x")

    def do_ee_py(self, arg):
        'Move EE +0.1m along y [u]'
        self._pub('u')
        print("move ee +0.1 y")

    def do_ee_ny(self, arg):
        'Move EE -0.1m along y [o]'
        self._pub('o')
        print("move ee -0.1 y")

    # ------------- mobile jog ---------------------------------------------
    def do_mobile_px(self, arg):
        'Move mobile +1.5m along x [a]'
        self._pub('a')
        print("move mobile +1.5x")

    def do_mobile_nx(self, arg):
        'Move mobile -1.5m along x [s]'
        self._pub('s')
        print("move mobile -1.5x")

    # ------------- whole body motion --------------------------------------
    def do_wb_global(self, arg):
        'Wholebody: +0.3x -0.1z EE in global coord [1]'
        self._pub('1')
        print("wholebody motion: delta in global")

    def do_wb_local(self, arg):
        'Wholebody: +0.2x +0.2y +0.1z EE in local coord [2]'
        self._pub('2')
        print("wholebody motion: delta in local")

    def do_chicken_p(self, arg):
        'Chicken head motion +mobile [3]'
        self._pub('3')
        print("chicken head motion with +0.1m mobile")

    def do_chicken_n(self, arg):
        'Chicken head motion -mobile [4]'
        self._pub('4')
        print("chicken head motion with -0.1m mobile")

    def do_mobile_back(self, arg):
        'Mobile base backward 5sec, maintain posture [5]'
        self._pub('5')
        print("mobile base backward during 5sec")

    def do_path_se3(self, arg):
        'Path-planner mobile motion + EE SE3 maintenance [6]'
        self._pub('6')
        print("path-planner mobile motion with EE SE3 maintenance")

    def do_path_mobile(self, arg):
        'Path-planner mobile-only motion + posture maintenance [7]'
        self._pub('7')
        print("path-planner mobile-only motion with posture maintenance")

    def do_wb_longrange(self, arg):
        'Long-range whole-body motion: -2.0x +3.0y -0.1z EE global [8]'
        self._pub('8')
        print("long-range whole-body motion")

    def do_nav2(self, arg):
        'Nav2 autonomous driving: base follows /ns1/cmd_vel, arm holds posture [9]'
        self._pub('9')
        print("Nav2 mode: follow /ns1/cmd_vel with arm posture maintenance")

    # ------------- cmd ----------------------------------------------------
    def do_estimation(self, arg):
        'Start object estimation [q]'
        self._pub('q')
        print("start estimation")

    def do_obj_dynamics(self, arg):
        'Toggle object dynamics [x]'
        self._pub('x')
        print("toggle object dynamics")

    def do_fext(self, arg):
        'Toggle external force application [b]'
        self._pub('b')
        print("toggle Fext application")

    def do_print_ee(self, arg):
        'Print current EE state [p]'
        self._pub('p')
        print("print current EE state")

    def do_grasp(self, arg):
        'Toggle grasp [z]'
        self._pub('z')
        print("toggle grasp")

    # ----------------------------------------------------------------------
    def do_quit(self, arg):
        'Exit the shell'
        return True


if __name__ == '__main__':
    shell = ControlSuiteShell()
    try:
        shell.cmdloop()
    finally:
        shell.node.destroy_node()
        rclpy.shutdown()
