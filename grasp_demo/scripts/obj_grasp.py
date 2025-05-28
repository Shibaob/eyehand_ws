#! /usr/bin/env python3

import tf
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Vector3, Wrench
from message_filters import ApproximateTimeSynchronizer, Subscriber
from vision_ros_msgs.msg import BoundingBoxes

class MoveRobot():
    def __init__(self):

        # 初始化 planning group
        self.robot = moveit_commander.robot.RobotCommander()
        self.arm_group = moveit_commander.move_group.MoveGroupCommander(
            "panda_arm")

        self.gripper_group=moveit_commander.move_group.MoveGroupCommander("hand")

        # 设置机械手臂的速度和加速度
        self.arm_group.set_max_acceleration_scaling_factor(0.2)
        self.arm_group.set_max_velocity_scaling_factor(0.2)

        # 物体的位置
        self.Obj_pose = PoseStamped()
        self.Obj_pose.pose.position.x = 0
        self.find_enable = False
        self.move_finish = False

        self.Obj_class = None

        self.obj_pose_sub = Subscriber(
            "/objection_position_pose", Pose)
        
        self.opencv_sub = Subscriber("vision_opencv", BoundingBoxes)

		# Sync Subscribers
        self.ats = ApproximateTimeSynchronizer(
			[
				self.obj_pose_sub,
				self.opencv_sub
			],
			queue_size=5,
			slop=1,
			allow_headerless=True
		)

        self.ats.registerCallback(self.ObjectCallback)

    def ObjectCallback(self, msg, opencv_msg):
        if self.find_enable:
            self.Obj_pose.pose = msg
            self.Obj_class = opencv_msg.bounding_boxes[0].Class
            print(self.Obj_class)

        if self.Obj_pose.pose.position.x != 0 :
            self.find_enable = False

    def stop(self):
        moveit_commander.roscpp_initializer.roscpp_shutdown()

    def plan_cartesian_path(self, pose):
            """
            笛卡尔路径规划

            Parameters:
                pose - 目标pose

            Returns:
                None
            """
            waypoints = []
            waypoints.append(pose)

            # 设置机器臂当前的状态作为运动初始状态
            self.arm_group.set_start_state_to_current_state()

            # 计算轨迹
            (plan, fraction) = self.arm_group.compute_cartesian_path(
                waypoints,   # waypoint poses，路点列表，这里是5个点
                0.01,        # eef_step，终端步进值，每隔0.01m计算一次逆解判断能否可达
                0.0,         # jump_threshold，跳跃阈值，设置为0代表不允许跳跃
                False)        # avoid_collisions，避障规划

            self.arm_group.execute(plan, wait=True)

    def goSP(self):

        self.arm_group.set_named_target('ready')
        self.arm_group.go(wait=True)

    def grasp_obj(self):

        # 去到物体上方位置
        print(self.Obj_pose)

        self.Obj_pose.pose.orientation = self.arm_group.get_current_pose().pose.orientation
        self.Obj_pose.pose.position.z += 0.10

        self.arm_group.set_pose_target(self.Obj_pose.pose)
        self.arm_group.go()

        # self.Obj_pose.pose.position.z -= 0.10
        # self.plan_cartesian_path(self.Obj_pose.pose)

    def main_loop(self):
        try:
            self.goSP()  # 1. 去到预抓取位置
            self.gripper_group.set_named_target("open")
            self.gripper_group.go()
            self.find_enable = True
            rospy.sleep(3)  # 2. 识别当前的抓取位姿态(3s)
            if self.find_enable == False:
                self.grasp_obj()
                self.gripper_group.set_named_target("close")
                self.gripper_group.go()
                self.move_finish=True
                self.Obj_pose.pose.position.x = 0
            else:
                rospy.logwarn('cant find object')
            
            self.goSP()
            self.move_finish = True

        except Exception as e:
            rospy.logerr(str(e))


def main():
    rospy.init_node('grasp_demo', anonymous=True)
    rospy.loginfo('Start Grasp Demo')
    moverobot = MoveRobot()
    while(not rospy.is_shutdown() and not moverobot.move_finish):
        moverobot.main_loop()


if __name__ == "__main__":

    main()