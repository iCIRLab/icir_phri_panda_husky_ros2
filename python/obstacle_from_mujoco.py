#! /usr/bin/python3
"""
obstacle_from_mujoco.py

Print the path-planner obstacle list parsed out of a MuJoCo XML scene.

ROS2 노트 (ROS1과 달라진 점)
---------------------------
ROS1에서 이 파일은 노드였고, 파싱 결과를 rospy.set_param()으로 **전역 파라미터
서버**에 써 두면 컨트롤러가 getParam으로 읽어갔다. ROS2에는 전역 파라미터 서버가
없다 - 파라미터는 노드마다 따로 산다.

그래서 노드를 없애고, launch 시점에 파싱해서 컨트롤러 노드의 파라미터로 직접
넘기는 방식으로 바꿨다. MJCF는 실행 중 바뀌지 않는 정적 파일이라 굳이 런타임에
노드를 띄울 이유가 없고, 이 워크스페이스의 mujoco_sim.launch.py가 URDF를 다루는
방식과도 같다. 실행 순서 의존도 생기지 않는다.

파싱 로직은 icir_phri_panda_husky.mjcf 로 옮겼고 launch가 거기서 import한다
(ns1_path_simulation_w_viz.launch.py 참고). 이 스크립트는 씬을 바꿨을 때 값이
뭐가 나오는지 눈으로 확인하는 디버깅용으로 남겨둔다:

    ros2 run icir_phri_panda_husky obstacle_from_mujoco.py <scene.xml> [--prefix obs]
"""
import argparse

from icir_phri_panda_husky.mjcf import parse_obstacles, parse_obstacles_flat


def main():
    parser = argparse.ArgumentParser(
        description='Print the obstacles parsed out of a MuJoCo XML scene.')
    parser.add_argument('model_file', help='path to the MuJoCo XML scene')
    parser.add_argument('--prefix', default='obs',
                        help='geom-name prefix marking obstacles (default: obs)')
    args = parser.parse_args()

    obstacles = parse_obstacles(args.model_file, args.prefix)
    print("obstacles            : %r" % (obstacles,))
    print("flat (launch에 넘길 것): %r" % (parse_obstacles_flat(args.model_file, args.prefix),))


if __name__ == '__main__':
    main()
