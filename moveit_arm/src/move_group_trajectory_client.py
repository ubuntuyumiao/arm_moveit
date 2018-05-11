#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import roslib
roslib.load_manifest('moveit_tutorials')
roslib.load_manifest('my_dynamixel_tutorial')
import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math


class Joint:                                   # 定义四个关节是一个类
        def __init__(self, motor_name):                                # 初始化函数
            #arm_name 
            self.name = motor_name           
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)   # 客户端，ActionServer名为'/'+self.name+'_controller/follow_joint_trajectory', slef.name is arm
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')

        def move_joint(self, angles):                                             # angles类型是元组，实参是所有关节转动的角度
            goal = FollowJointTrajectoryGoal()                  
            #joint name should be the same as the tilt.yaml
            goal.trajectory.joint_names = ['Joint1', 'Joint2','Joint3','Joint4']           
            point = JointTrajectoryPoint()
            point.positions = angles                                              # 得到每个关节点的位置
            point.time_from_start = rospy.Duration(3)                   
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)

        def move_arm(self,traget_pose):
            moveit_commander.roscpp_initialize(sys.argv)
            robot = moveit_commander.RobotCommander()
            scene = moveit_commander.PlanningSceneInterface()
            group = moveit_commander.MoveGroupCommander('arm')
            group.set_pose_target(traget_pose)
            plan = group.plan()                                    # 规划
            print plan
            if plan:
                print "-------------find result"
                points = plan.joint_trajectory.points
                for point in points:
                    position=point.positions
                    realposition=[5*math.pi/6-position[0],math.pi+position[1],5*math.pi/6-position[2], 5*math.pi/6+position[3]]
                    print "----------set realposition"
                    print realposition
                    self.move_joint(realposition)
	    else:
		print "------------not find--"

        def reset_position(self):
            self.move_joint([5*math.pi/6, math.pi, 5*math.pi/6, 5*math.pi/6])


                        
if __name__ == '__main__':
        rospy.init_node('group_move')
        arm = Joint('arm')                                # 实例化一个Joint类
        arm.reset_position()                              # 关节复位
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation.w = -0.0323522
        pose_target.orientation.x = 0.239105
        pose_target.orientation.y = 0.830536
        pose_target.orientation.z = 0.501989
        pose_target.position.x = -0.0337157
        pose_target.position.y = -0.0706073
        pose_target.position.z = 0.780014
        try:
            arm.move_arm(pose_target)
        except rospy.ROSInterruptException:
            pass
