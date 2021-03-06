#!/usr/bin/python
from core_tool import *
def Help():
  return '''Move arm to follow a target joint-angular trajectory.
  Warning: Be careful to moving area of robot.
  Usage: ex.follow_q_traj1
  '''
def Run(ct,*args):
  arm= RIGHT
  dof= ct.robot.DoF(arm)
  if dof>7:  raise Exception('Does not work with DoF>7.')
  q= list(ct.robot.Q(arm=arm))  #Current joint angles
  q_traj= [
      q,
      [q[d]+(0.1,0.0,0.0,0.0,0.0,0.0,0.0)[d] for d in range(dof)],
      [q[d]+(-0.1,0.0,0.0,0.0,0.0,0.0,0.0)[d] for d in range(dof)],
      [q[d]+(0.0,0.1,0.0,0.0,0.0,0.0,0.0)[d] for d in range(dof)],
      [q[d]+(0.0,-0.1,0.0,0.0,0.0,0.0,0.0)[d] for d in range(dof)],
      q
    ]
  t_traj= [0.0, 3.0, 6.0, 9.0, 12.0, 15.0]
  ct.robot.FollowQTraj(q_traj, t_traj, arm=arm)
