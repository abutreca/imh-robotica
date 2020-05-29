#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import copy
from tf import transformations
import geometry_msgs.msg
#from geometry_msgs.msg import Pose

rospy.init_node( 'move' )

robot= moveit_commander.RobotCommander()
ur_group= moveit_commander.MoveGroupCommander('manipulator')

print "=========== Reference frame: %s" % ur_group.get_planning_frame()
print "=========== End effector frame: %s" % ur_group.get_end_effector_link()
print "=========== Robot Groups: %s" % ", ".join(robot.get_group_names())
print "=========== Printing robot state"
print robot.get_current_state()
print "=========== Current cartesian pose: %s" % ur_group.get_current_pose()
print "=========== Current joint values: %s" % ur_group.get_current_joint_values()
#raw_input('Pause')

print "How many objects need to be picked up?"
n=input()
print "How many rows in the grid?"
n_y=input()
print "How many columns in the grid?"
n_x=input()
print "Distance between rows in the grid?"
dy=input()
print "Distance between columns in the grid?"
dx=input()
print "Set initial X position:"
x0=input()
print "Set initial Y position:"
y0=input()

i=0
j=0
k=0
x=x0
y=y0


print "Moving to safe pose in joint space"
safe_position = [-1.57, -1.04, 1.3, -1.57, -1.57, 0]
ur_group.set_joint_value_target(safe_position)
plan = ur_group.plan()
ur_group.execute( plan )
print "In safe position"

while i<n:
  print "Moving in cartesian space to approach position"
  pose = [0.95, 0, 1.2, 0, 1.57, 0]
  ur_group.set_pose_target( pose )
  plan = ur_group.plan()
  ur_group.execute( plan ) 
  rospy.sleep(3)

  print "Moving in cartesian space to pick position"
  pose = [0.95, 0, 0.75, 0, 1.57, 0]
  ur_group.set_pose_target( pose )
  plan = ur_group.plan()
  ur_group.execute( plan ) 
	  
  print "Moving in cartesian space back to approach position"
  pose = [0.95, 0, 1.2, 0, 1.57, 0]
  ur_group.set_pose_target( pose )
  plan = ur_group.plan()
  ur_group.execute( plan ) 
  rospy.sleep(3)
  if k<n_y:
    if j<n_x:
      print "Moving in cartesian space to approach position in placing side"
      pose = [x, y, 1.2, 1.57, 1.57, 1.57]
      ur_group.set_pose_target( pose )
      plan = ur_group.plan()
      ur_group.execute( plan ) 
      rospy.sleep(3)
      
      print "Moving in cartesian space to place position"
      pose = [x, y, 0.75, 1.57, 1.57, 1.57]
      ur_group.set_pose_target( pose )
      plan = ur_group.plan()
      ur_group.execute( plan ) 
	  
      print "Moving in cartesian space back to approach position"
      pose = [x, y, 1.2, 1.57, 1.57, 1.57]
      ur_group.set_pose_target( pose )
      plan = ur_group.plan()
      ur_group.execute( plan ) 
      rospy.sleep(3)
      j+=1
      x-=dx
      
    elif j==n_x:
	  k+=1
	  if k==n_y:
	    if i<n:
	      print "Moving to safe pose in joint space"
	      safe_position = [-1.57,-1.04, 1.3, -1.57, -1.57, 0]
	      ur_group.set_joint_value_target(safe_position)
	      plan=ur_group.plan()
	      ur_group.execute( plan )
	      print "In safe position after failure"
	      sys.exit("Need more grid")
	  j=1
	  x=x0
	  y-=dy
	  print "Moving in cartesian space to approach position in placing side"
	  pose = [x, y, 1.2, 1.57, 1.57, 1.57]
	  ur_group.set_pose_target( pose )
	  plan = ur_group.plan()
	  ur_group.execute( plan )
	  rospy.sleep(3)
	  
	  print "Moving in cartesian space to place position"
	  pose=[x,y,0.75,1.57,1.57,1.57]
	  plan=ur_group.plan()
	  ur_group.execute( plan )
	  
	  print "Moving in cartesian space back to approach position"
	  pose=[x,y,1.2,1.57,1.57,1.57]
	  ur_group.set_pose_target( pose )
	  plan=ur_group.plan()
	  ur_group.execute( plan)
	  rospy.sleep(3)
	  
  i+=1
  print "Object  %d has being moved" % i
  
print "Moving to safe pose in joint space"
safe_position = [-1.57, -1.04, 1.3, -1.57, -1.57, 0]
ur_group.set_joint_value_target(safe_position)
plan = ur_group.plan()
ur_group.execute( plan )
print "In safe position again"
 
 
