#!/usr/bin/env python
import roslib

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal

roslib.load_manifest('my_dynamixel_tutorial')


class Joint:
        def __init__(self, motor_name):
            #arm_name 
            self.name = motor_name           
            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')

            
        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()                  
            #joint name should be the same as the tilt.yaml
            goal.trajectory.joint_names = ['Joint1', 'Joint2','Joint3','Joint4']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(3)                   
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)
              

def main():
            #arm name should be the same as the joints_trajectory_controller.yaml
            arm = Joint('arm')
            arm.move_joint([0.0,3.0,2.5,1.0])
            arm.move_joint([0.2,3.2,2.4,0.8])

                        
if __name__ == '__main__':
            rospy.init_node('joint_position_tester')
            main()
