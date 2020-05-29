#!/usr/bin/env python

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
i=1

print "Moving to safe pose in joint space"
safe_position = [-1.57, -1.04, 1.3, -1.57, -1.57, 0]
ur_group.set_joint_value_target(safe_position)
#rospy.loginfo( 'Planning movement' )
plan = ur_group.plan()
#rospy.loginfo( 'Executing plan' )
ur_group.execute( plan )
raw_input('In safe position')

while i<n+1:
  print "Moving in cartesian space to approach position"
  pose = [1.15, 0, 1.2, 0, 1.57, 0]
  ur_group.set_pose_target( pose )
  #rospy.loginfo( 'Planning movement' )
  plan = ur_group.plan()
  #rospy.loginfo( 'Executing plan' )
  ur_group.execute( plan ) 
  rospy.sleep(3)

  print "Moving in cartesian space to pick position"
  pose = [1.15, 0, 0.75, 0, 1.57, 0]
  ur_group.set_pose_target( pose )
  #rospy.loginfo( 'Planning movement' )
  plan = ur_group.plan()
  #rospy.loginfo( 'Executing plan' )
  ur_group.execute( plan ) 

  print "Moving in cartesian space back to approach position"
  pose = [1.15, 0, 1.2, 0, 1.57, 0]
  ur_group.set_pose_target( pose )
  #rospy.loginfo( 'Planning movement' )
  plan = ur_group.plan()
  #rospy.loginfo( 'Executing plan' )
  ur_group.execute( plan ) 
  rospy.sleep(3)

  print "Moving in cartesian space back to place position"
  pose = [0, 0, 0.75, 0, 1.57, 0]
  ur_group.set_pose_target( pose )
 #rospy.loginfo( 'Planning movement' )
  plan = ur_group.plan()
 #rospy.loginfo( 'Executing plan' )
  ur_group.execute( plan ) 

  print "Object  %d has being moved" % i
  i+=1

print "Moving to safe pose in joint space"
safe_position = [-1.57, -1.04, 1.3, -1.57, -1.57, 0]
ur_group.set_joint_value_target(safe_position)
#rospy.loginfo( 'Planning movement' )
plan = ur_group.plan()
#rospy.loginfo( 'Executing plan' )
ur_group.execute( plan )
print "In safe position again"
