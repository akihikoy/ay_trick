#!/usr/bin/python
from core_tool import *
def Help():
  return '''Demo of moving to a fixed target location, pick up with FV, and openif.
  Usage:
    fv.move_pick_demo
'''

def Run(ct,*args):
  arm= 0
  lw_xf= ct.GetAttr('wrist_r','lx')
  xf_grasp= [0.3001751420310972, 0.0034782857844863663, 0.13108151343221114, -0.0006227473634600952, 0.6348255443482752, 0.0005493296214830417, 0.7726550580081674]
  xf_give= [0.4273953291726786, 0.017801955296955253, 0.2752075465354592, -1.4368092744227403e-05, 0.5202139762099631, 0.0003873556121128884, 0.8540358708537994]
  xf_gup= xf_grasp+np.array([0,0,0.08, 0,0,0,0])
  g_pregrasp= 0.08
  g_grasp= 0.04

  ct.robot.MoveGripper(g_pregrasp, arm=arm, blocking=True)
  ct.robot.MoveToX(xf_gup, x_ext=lw_xf, dt=6.0, arm=arm, blocking=True)
  ct.robot.MoveToX(xf_grasp, x_ext=lw_xf, dt=6.0, arm=arm, blocking=True)
  ct.robot.MoveGripper(g_grasp, arm=arm, blocking=True)
  rospy.sleep(1.0)
  ct.Run('fv.pickup2b', 'once', arm)
  for i in range(20):
    if ct.robot.actc.traj.get_state()==3:  break
    print 'Robot is not in normal state.'
    rospy.sleep(0.25)
  if ct.robot.actc.traj.get_state()!=3:  return
  ct.robot.MoveToX(xf_give, x_ext=lw_xf, dt=6.0, arm=arm, blocking=True)
  ct.Run('fv.openif','on',arm)
  rospy.sleep(6.0)

