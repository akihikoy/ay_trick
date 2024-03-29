#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move the fingertip to a target pose with Cartesian interpolation.
  Warning: Be careful to moving area of robot.
  Usage: movexe POSE [, DURATION [, LX]]
    POSE: Pose vector, which can be:
      [x,y,z]:  Change position only.
      [quaternion]:  Change orientation only.
      [x,y,z, quaternion]:  Change pose.
    DURATION: Duration of motion.  Default: 4.0
    LX: Fingertip pose in the wrist frame. Default: None
      If None, we use the attribute: 'wrist_'+LRToStrs(ct.robot.Arm),'lx' '''
def Run(ct,*args):
  xe_trg= args[0]
  dt= args[1] if len(args)>1 else 4.0
  lx_f= args[2] if len(args)>2 else None
  if lx_f is None: lx_f= ct.GetAttr('wrist_'+LRToStrs(ct.robot.Arm),'lx')  #Fingertip pose (local pose in the wrist frame).
  xe= list(ct.robot.FK(x_ext=lx_f))
  if len(xe_trg)==3:  xe_trg= list(xe_trg)+xe[3:]
  elif len(xe_trg)==4:  xe_trg= xe[:3]+list(xe_trg)
  assert(len(xe_trg)==7)
  print 'Move to xe:',xe_trg
  ct.robot.MoveToXI(xe_trg, dt, x_ext=lx_f)
