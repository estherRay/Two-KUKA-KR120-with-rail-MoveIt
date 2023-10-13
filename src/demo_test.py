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
def display_traj(self, publisher, plan):
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan)
  # Publish
  publisher.publish(display_trajectory);
    
  
# Create Plan for Move Group Function #####################
def create_plan(self, move_group, scale, X, Y, Z):
  waypoints = []
  
  wpose = move_group.get_current_pose().pose
  wpose.position.x -= scale * X # First, move down (z)
  wpose.position.y += scale * Y # x in positive direction
  wpose.position.z += scale * Z # y in positive direction
  waypoints.append(copy.deepcopy(wpose))

  (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

  # Just planning, not moving Panda yet:
  return plan, fraction
  
  
# Execute Plan for Move Group Function #####################
def execute_plan(self, move_group, plan):
  move_group.execute(plan, wait=True)
  
  
# Execute Plan for RAIL #####################
def execute_rail_plan(self, x):
  goal = rail.get_current_joint_values()
  goal[0]=x
  rail.go(goal, wait=True)
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
  while i < 20:
    print ("===== Loop: ", i, "  Stage 1: SOL moves")
    # Give positions to SOL ARM's EF to go to
    plan0, fraction0 = create_plan(sol_arm, move_group=sol_arm, scale=1, X=0.2, Y=0, Z=-0.2)
    rospy.sleep(3)
    display_traj(sol_arm_pub, publisher=sol_arm_pub, plan=plan0)
    execute_plan(sol_arm, move_group=sol_arm, plan=plan0)
    rospy.sleep(2) # wait
    sol_arm.stop() # Stop the current moving group
    
    print ("===== Stage 2: RAIL moves")
    # Give a value to RAIL to go to
    execute_rail_plan(rail, -5.0)
    rospy.sleep(2) # wait
    rail.stop()  # Stop the current moving group
    
    print ("===== Stage 3: MANI moves")
    # Give positions to SOL ARM's EF to go to
    plan1, fraction1 = create_plan(mani_arm, move_group=mani_arm, scale=1, X=0.4, Y=0.2, Z=+0.4)
    display_traj(mani_arm_pub, publisher=mani_arm_pub, plan=plan1)
    execute_plan(mani_arm, move_group=mani_arm, plan=plan1)
    rospy.sleep(2) # wait
    mani_arm.stop() # Stop the current moving group
    
    print ("===== Stage 4: SOL moves")
    # Give positions to SOL ARM's EF to go to
    plan0, fraction0 = create_plan(sol_arm, move_group=sol_arm, scale=1, X=-0.6, Y=0.0, Z=0.6)
    rospy.sleep(3)
    display_traj(sol_arm_pub, publisher=sol_arm_pub, plan=plan0)
    execute_plan(sol_arm, move_group=sol_arm, plan=plan0)
    rospy.sleep(2) # wait
    sol_arm.stop() # Stop the current moving group
    
    print ("===== Stage 5: RAIL moves")
    # Give a value to RAIL to go to
    execute_rail_plan(rail, -15.0)
    rospy.sleep(2) # wait
    rail.stop()  # Stop the current moving group

    print ("===== Stage 6: MANI moves")
    # Give positions to SOL ARM's EF to go to
    plan1, fraction1 = create_plan(mani_arm, move_group=mani_arm, scale=1, X=0.6, Y=0.3, Z=-0.6)
    display_traj(mani_arm_pub, publisher=mani_arm_pub, plan=plan1)
    execute_plan(mani_arm, move_group=mani_arm, plan=plan1)
    rospy.sleep(2) # wait
    mani_arm.stop() # Stop the current moving group
    
    print ("===== Stage 7: SOL moves")
    # Give positions to SOL ARM's EF to go to
    plan0, fraction0 = create_plan(sol_arm, move_group=sol_arm, scale=1, X=+0.2, Y=0.0, Z=0.2)
    rospy.sleep(3)
    display_traj(sol_arm_pub, publisher=sol_arm_pub, plan=plan0)
    execute_plan(sol_arm, move_group=sol_arm, plan=plan0)
    rospy.sleep(2) # wait
    sol_arm.stop() # Stop the current moving group
    
    print ("===== Stage 8: RAIL moves")
    # Give a value to RAIL to go to
    execute_rail_plan(rail, -0.0)
    rospy.sleep(2) # wait
    rail.stop()  # Stop the current moving group
    
    print ("===== Stage 9: MANI moves")
    # Give positions to SOL ARM's EF to go to
    plan1, fraction1 = create_plan(mani_arm, move_group=mani_arm, scale=1, X=0.2, Y=0.0, Z=0.3)
    display_traj(mani_arm_pub, publisher=mani_arm_pub, plan=plan1)
    execute_plan(mani_arm, move_group=mani_arm, plan=plan1)
    rospy.sleep(2) # wait
    mani_arm.stop() # Stop the current moving group
    i+=1  # Loop through the demo
    
  # Once loop is completed, exit
  exit()
