#!/usr/bin/env python3
import moveit_commander
import rospy
import os
# import tf
import geometry_msgs.msg
import numpy as np
import copy
import tf

# moveit library
robot = moveit_commander.RobotCommander()
xarm = moveit_commander.MoveGroupCommander("xarm6")
psi = moveit_commander.PlanningSceneInterface()

if os.name == 'nt':
    import msvcrt

    def getch():
        return msvcrt.getch().decode()
else:
    import sys
    import tty
    import termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def ArmInitialization():
    print("=" * 10, " Robot Groups:")
    print(robot.get_group_names())
    print("=" * 10, " Printing robot state")
    print(robot.get_current_state())
    print("=" * 15, " xarm ", "=" * 15)
    print("=" * 10, " Reference frame: %s" %
          xarm.get_planning_frame())
    print("=" * 10, " Reference frame: %s" %
          xarm.get_end_effector_link())
    xarm_initial_pose = xarm.get_current_pose()
    print("=" * 10, " Printing Xarm initial pose: ")
    print(xarm_initial_pose.pose)

    xarm_initial_rpy = xarm.get_current_rpy()
    print("=" * 10, " Printing Xarm initial rpy: ")
    print(xarm_initial_rpy)

    xarm.set_max_velocity_scaling_factor(0.5)
    xarm.set_max_acceleration_scaling_factor(0.5)

    return xarm_initial_pose.pose


def Go_homeposition_EE():
    home_pose = geometry_msgs.msg.Pose()
    home_pose.position.x =  0.20701760221168963
    home_pose.position.y = 1.4408959667063535e-05
    home_pose.position.z = 0.03400251924164405
    home_pose.orientation.x = -0.9999072837831338
    home_pose.orientation.y = -4.859141964351074e-05
    home_pose.orientation.z = 0.013613290340705797
    home_pose.orientation.w = 0.00031591519361944936

    xarm.set_max_velocity_scaling_factor(0.6)

    xarm.set_pose_target(home_pose)
    xarm.go()

def Go_homeposition():
    home_pose = geometry_msgs.msg.Pose()
    home_pose.position.x =  0.20701760221168963
    home_pose.position.y = 1.4408959667063535e-05
    home_pose.position.z = 0.11199180862771843
    home_pose.orientation.x = -0.9999072837831338
    home_pose.orientation.y = -4.859141964351074e-05
    home_pose.orientation.z = 0.013613290340705797
    home_pose.orientation.w = 0.00031591519361944936

    xarm.set_max_velocity_scaling_factor(0.6)

    xarm.set_pose_target(home_pose)
    xarm.go()


# debug
def armpose_checker():
    pose=xarm.get_current_pose().pose
    print("=" * 10, " Printing Xarm current pose: ")
    print(pose)
    rpy = xarm.get_current_rpy()
    print("=" * 10, " Printing Xarm current rpy: ","="*10)
    print(rpy)

def test_pose():
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.300
    target_pose.position.y = 0.400
    target_pose.position.z = 0.200
    target_pose.orientation.x = -0.9999072837831338
    target_pose.orientation.y = -4.859141964351074e-05
    target_pose.orientation.z = 0.013613290340705797
    target_pose.orientation.w = 0.00031591519361944936
    
    xarm.set_pose_target(target_pose)
    xarm.go()

"""課題1"""
def input_pose():
    print("x,y,z[mm] >>")
    x,y,z = map(int, input().split())  

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = float(x/1000)
    target_pose.position.y = float(y/1000)
    target_pose.position.z = float(z/1000)
    target_pose.orientation.x = -0.9999072837831338
    target_pose.orientation.y = -4.859141964351074e-05
    target_pose.orientation.z = 0.013613290340705797
    target_pose.orientation.w = 0.00031591519361944936
    
    xarm.set_pose_target(target_pose)
    xarm.go()

"""課題2"""
def spawn_box():
    box_size=(0.085,0.085,0.260)
    box_pose=geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id=xarm.get_planning_frame()
    box_pose.pose.position.x=box_size[0]/2 +0.250
    box_pose.pose.position.y=box_size[1]/2 +0.200
    box_pose.pose.position.z=box_size[2]/2 
    psi.add_box("box", box_pose,box_size)

"""課題3"""
def circle():
    wpts = []
    target_pose = geometry_msgs.msg.Pose()

    center=[0.3,0.4,0.2]
    radius = 0.05
    steps = 30
    rpy=[np.pi,0.0,0.0]

    quat = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    target_pose.orientation.x = quat[0]
    target_pose.orientation.y = quat[1]
    target_pose.orientation.z = quat[2]
    target_pose.orientation.w = quat[3]
    step_rad=2*np.pi/steps

    for i in range(3*steps+1):
        ang_rad=step_rad*i
        target_pose.position.x = center[0] - radius* np.cos(ang_rad)
        target_pose.position.y = center[1] + radius* np.sin(ang_rad)
        target_pose.position.z = center[2]
        wpts.append(copy.deepcopy(target_pose))
    (plan, fraction) = xarm.compute_cartesian_path(wpts, 0.01, 0.0)
    if fraction<1.0:
        print("Success path ",fraction)
        print("Planning False")
        quit()
    # print (fraction)
    xarm.execute(plan)


"""課題5"""
def grasp():
    #ハンドにPETのモデルをくっつける
    xarm.attach_object("PET", link_name="xarm_gripper_base_link")
    rospy.sleep(3)

def openhand():
    xarm.detach_object("PET", link_name="xarm_gripper_base_link")
    psi.remove_world_object(name=None)
    rospy.sleep(3)

def add_cylinder():
    pet_size=(0.036,0.260)
    pet_pose=geometry_msgs.msg.PoseStamped()
    pet_pose.header.frame_id=xarm.get_planning_frame()
    pet_pose.pose.position.x=0.300
    pet_pose.pose.position.y=0.400
    pet_pose.pose.position.z=pet_size[1]/2 
    psi.add_cylinder("PET", pet_pose, pet_size[1], pet_size[0])
    return pet_pose

def approach(pet_pose):

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = pet_pose.pose.position.x -0.03
    target_pose.position.y = pet_pose.pose.position.y -0.17
    target_pose.position.z = pet_pose.pose.position.z +0.0

    rpy=[1.5*np.pi,0.0*np.pi,0.5*np.pi]
    quat = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
    target_pose.orientation.x = quat[0]
    target_pose.orientation.y = quat[1]
    target_pose.orientation.z = quat[2]
    target_pose.orientation.w = quat[3]
    xarm.set_pose_target(target_pose)
    xarm.go()

def transport():
    pet_pose=add_cylinder()
    approach(pet_pose)
    grasp()
    Go_homeposition_EE()
    openhand()


"""
課題にある動作を実現する関数を作ってみよう.
なお、必ずGazeboシミュレーション上で動作を確認してから実機でチェックを行うこと.
moveitの関数の使い方の説明等は以下のURLを見れば分かります.
URL:https://robo-marc.github.io/moveit_documents/moveit_commander.html

gazeboでシミュレーションする方法:
ターミナル
$ roslaunch xarm_gazebo xarm6_beside_table.launch
別ターミナル
$ roslaunch xarm6_moveit_config xarm6_moveit_gazebo.launch

実機で動作させる場合:
$ roslaunch xarm6_moveit_config realMove_exec.launch robot_ip:=192.168.1.217 

"""


if __name__ == '__main__':
    # connect robot arm (xArm6)
    rospy.init_node("xArm_tutorial")
    ArmInitialization()

    while True:
        print("************* Command ***************")
        print("c:アームの状態取得")
        print("q:終了")
        print("t:テストポーズ")
        print("h:ホームポジションに戻る")
        print("r:remove object")
        print("1:課題1")
        print("2:課題2")
        print("3:課題3")
        # print("4:課題4")
        print("5:課題5")

        mode = input("mode select>>")
        print("You select mode : %s " % mode)

        if mode == "c":
            armpose_checker()

        elif mode == "q":
            quit()

        elif mode=="t":
            test_pose()

        elif mode=="h":
            Go_homeposition()
        
        elif mode=="r":
            psi.remove_world_object(name=None)
        
        elif mode=="1":
            input_pose()

        elif mode=="2":
            spawn_box()
            input_pose()
        
        elif mode=="3":
            circle()
        
        elif mode=="5":
            transport()

        else:
            print("there isn't such mode")


