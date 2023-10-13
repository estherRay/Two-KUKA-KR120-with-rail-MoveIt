#!/usr/bin/env python3

# This is a simple test to ensure this kind of program works with MoveIt KUKA ...

#   ... and it does!  (at least in simulation)

import rospy
import tf
import sys
import copy
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
import actionlib

# INITIALISATION ###############################
# Initialize moveit commander
moveit_commander.roscpp_initialize(sys.argv)
# Initialize node
rospy.init_node('move_sim_test', anonymous=True)
# Get the robot used
robot = moveit_commander.RobotCommander()
# Planning Scene
scene = moveit_commander.PlanningSceneInterface()
# Rate
rate = rospy.Rate(1)

# SET UP MOVE GROUPS #############################
# SOL ARM Move GROUP
sol_arm = moveit_commander.MoveGroupCommander("Sol_arm")
sol_arm_joints = sol_arm.get_joints()
sol_arm_pub = rospy.Publisher('/sol_arm/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory, queue_size=10)
# SOL HAND Move GROUP
sol_hand = moveit_commander.MoveGroupCommander("Sol_hand")
sol_hand_joints = sol_hand.get_joints()
sol_hand_pub = rospy.Publisher('/sol_hand/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory, queue_size=10)
# MANI ARM Move GROUP
mani_arm = moveit_commander.MoveGroupCommander("Mani_arm")
mani_arm_joints = mani_arm.get_joints()
mani_arm_pub = rospy.Publisher('/mani_arm/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory, queue_size=10)
# MANI HAND Move GROUP
mani_hand = moveit_commander.MoveGroupCommander("Mani_hand")
mani_hand_joints = mani_hand.get_joints()
mani_hand_pub = rospy.Publisher('/mani_hand/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory, queue_size=10)
# RAIL Move GROUP
rail = moveit_commander.MoveGroupCommander("Rail")
rail_joints = rail.get_joints()
rail_pub = rospy.Publisher('/rail/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory, queue_size=10)


# Display Trajectory Function ######################
def display_traj(self, plan):
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan)
  # Publish
  sol_arm_pub.publish(display_trajectory);
  
  
# Execute Plan for Move Group Function #####################
def execute_plan(self, plan):
  sol_arm.execute(plan, wait=True)
  
  
# Create Plan for Move Group Function #####################
def create_plan(self, scale=1):
  waypoints = []

  wpose = sol_arm.get_current_pose().pose
  wpose.position.z -= scale * 0.1 # First, move down (z)
  wpose.position.x += scale * 0.2 # x in positive direction
  wpose.position.y += scale * 0.0 # y in positive direction
  waypoints.append(copy.deepcopy(wpose))

  (plan, fraction) = sol_arm.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

  # Just planning, not moving Panda yet:
  return plan, fraction
  
# Execute Plan for RAIL #####################
def execute_rail_plan(self):
  rail_goal = rail.get_current_joint_values()
  rail_goal[0]=-10.0
  rail.go(rail_goal, wait=True)
  rail.stop()
  
# MAIN ##############################################
if __name__ == '__main__':
  # Print info
  print ("===== Available Planning Groups:", robot.get_group_names())
  
  # Allow replanning if some fail attempts happens
  sol_arm.allow_replanning(True)
  mani_arm.allow_replanning(True)
  
  # Add obstacles to planning scene
  p = PoseStamped()
  p.header.frame_id = robot.get_planning_frame()
  p.pose.position.x = 0.0
  p.pose.position.y = -5.0
  p.pose.position.z = -0.01
  scene.add_box("ground", p, (10, 30, 0.01))  #(name, pose, geometry)
  rospy.sleep(1) # time to process info
  
  i = 0
  while i < 2:
    print ("===== Stage 1: Getting into first pose")
    plan, fraction = create_plan(sol_arm,1)
    display_traj(sol_arm_pub, plan)
    execute_plan(sol_arm, plan)
    rospy.sleep(2)
    sol_arm.stop()
    print ("===== Stage 2: Move rail")
    execute_rail_plan(rail)
    rospy.sleep(2)
    rail.stop()
    i+=1
  exit()
