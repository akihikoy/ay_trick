#!/usr/bin/python3
from core_tool import *
def Help():
  return '''Demo of moving to a fixed target location, pick up with FV, and openif.
  Usage:
    fv.move_pick_demo
'''

def Run(ct,*args):
  arm= 0
  lw_xf= ct.GetAttr('wrist_r','lx')
  xf_grasp= [0.34296284671072086, -0.07405146955823912, 0.13099378847801926, -0.0008455844514294216, 0.6346613809089622, 0.00037699547956107784, 0.7727897996506378]
  xf_give= [0.45968911233497334, -0.08046464510158438, 0.2752295841485144, -0.0003589051311227776, 0.5197019098781692, 5.7401697759803405e-06, 0.8543475850162745]
  xf_gup0= xf_grasp+np.array([0,0,0.03, 0,0,0,0])
  xf_gup= xf_grasp+np.array([0,0,0.10, 0,0,0,0])
  g_pregrasp= 0.08
  g_grasp= 0.04

  ct.robot.MoveGripper(g_pregrasp, arm=arm, blocking=True)
  try:
    ct.robot.MoveToX(xf_gup, x_ext=lw_xf, dt=4.0, arm=arm, blocking=True)
  except ROSError as e:
    CPrint(4,'Error: {}; tentatively ignored.'.format(e))
  try:
    ct.robot.MoveToX(xf_grasp, x_ext=lw_xf, dt=3.0, arm=arm, blocking=True)
  except ROSError as e:
    CPrint(4,'Error: {}; tentatively ignored.'.format(e))
  ct.robot.MoveGripper(g_grasp, arm=arm, blocking=True)
  rospy.sleep(0.5)
  
  #ct.Run('fv.pickup2b', 'once', arm)
  #for i in range(20):
    #if ct.robot.actc.traj.get_state()==3:  break
    #print 'Robot is not in normal state.'
    #rospy.sleep(0.25)
  #if ct.robot.actc.traj.get_state()!=3:  return
  ct.Run('fv.hold', 'on', arm)
  try:
    ct.robot.MoveToX(xf_gup0, x_ext=lw_xf, dt=3.0, arm=arm, blocking=True)
  except ROSError as e:
    CPrint(4,'Error: {}; tentatively ignored.'.format(e))
  try:
    ct.robot.MoveToX(xf_gup, x_ext=lw_xf, dt=3.0, arm=arm, blocking=True)
  except ROSError as e:
    CPrint(4,'Error: {}; tentatively ignored.'.format(e))
  ct.Run('fv.hold', 'off', arm)
  
  try:
    ct.robot.MoveToX(xf_give, x_ext=lw_xf, dt=4, arm=arm, blocking=True)
  except ROSError as e:
    CPrint(4,'Error: {}; tentatively ignored.'.format(e))
  ct.Run('fv.openif','on',arm)
  rospy.sleep(6.0)

