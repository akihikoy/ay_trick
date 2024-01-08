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
  xf_grasp= [0.396, 0.001, 0.084, -6.075900690479501e-05, 0.6008066570081552, 3.466204542040238e-05, 0.799394368257352]
  xf_give= [0.45075966415428, 0.001034045354837597, 0.19945473685548198, -5.421697556891315e-05, 0.521255723151148, 7.925840489524081e-05, 0.8534005283926175]
  xf_gup= xf_grasp+np.array([0,0,0.10, 0,0,0,0])
  g_pregrasp= 0.05
  g_grasp= 0.035

  ct.robot.MoveToXI(xf_gup, x_ext=lw_xf, dt=4.0, arm=arm, blocking=True)
  ct.robot.MoveGripper(g_pregrasp, arm=arm, blocking=True)
  ct.robot.MoveToXI(xf_grasp, x_ext=lw_xf, dt=3.0, arm=arm, blocking=True)
  ct.robot.MoveGripper(g_grasp, arm=arm, blocking=True)
  ct.Run('fv.pickup2b', 'once', arm)
  ct.robot.MoveToXI(xf_give, x_ext=lw_xf, dt=3.0, arm=arm, blocking=True)
  ct.Run('fv.openif','on',arm)

