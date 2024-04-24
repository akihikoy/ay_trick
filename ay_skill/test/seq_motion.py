#!/usr/bin/python
from core_tool import *
import industrial_msgs.msg

def Help():
  return '''Test of sequence of motions (repeating FollowQTraj).
  Usage:
  > test.seq_motion 'init' [, SPEED_GRAIN]
  > test.seq_motion 'run' [, SPEED_GRAIN  [, N_REPEAT]]
  '''

def PrintRobotState(robot, text=''):
  if robot.Is('Motoman'):
    print '{}: e_stopped={} drives_powered={} motion_possible={} in_error={} actc_st={}'.format(
      text,
      robot.robot_status.e_stopped.val==industrial_msgs.msg.TriState.FALSE,
      robot.robot_status.drives_powered.val==industrial_msgs.msg.TriState.TRUE,
      robot.robot_status.motion_possible.val==industrial_msgs.msg.TriState.TRUE,
      robot.robot_status.in_error.val==industrial_msgs.msg.TriState.FALSE,
      robot.actc.traj.get_state(),
      )

def Run(ct,*args):
  cmd= args[0] if len(args)>0 else None
  speed_gain= args[1] if len(args)>1 else 1.0
  n_repeat= args[2] if len(args)>2 else 1

  q_traj= [
    [0.0, -0.1299358457326889, -0.5056307315826416, 0.0, -0.8293323516845703, -3.7544981751125306e-05],
    [-0.30504995584487915, 0.011838496662676334, -0.37266528606414795, -0.1445828080177307, -0.8412446975708008, 0.3826960027217865],
    [-0.48530420660972595, -0.0716867595911026, -0.45356741547584534, -0.21923653781414032, -0.8742252588272095, 0.5995934009552002],
    [-0.6080750823020935, 0.03278883919119835, -0.35125091671943665, -0.2640083134174347, -0.8994401097297668, 0.7429025769233704],
    ]
  arm= 0
  dt= 4.0/speed_gain
  #blocking_mode= 'time'
  blocking_mode= True

  t_traj= [i*dt for i in range(len(q_traj))]
  q_traj_rev= list(reversed(q_traj))

  if cmd is None:
    print Help()
    print 'q_traj=',q_traj
    print 't_traj=',t_traj
    print 'q_traj_rev=',q_traj_rev
    return

  if cmd=='init':
    ct.robot.MoveToQ(q_traj[0], dt=5.0/speed_gain, arm=arm, blocking=blocking_mode)
  elif cmd=='run':
    for i in range(n_repeat):
      q_traj[0]= ct.robot.Q(arm=arm)
      PrintRobotState(ct.robot, 'p10')
      ct.robot.FollowQTraj(q_traj, t_traj, arm=arm, blocking=blocking_mode)
      PrintRobotState(ct.robot, 'p20')
      q_traj_rev[0]= ct.robot.Q(arm=arm)
      PrintRobotState(ct.robot, 'p21')
      ct.robot.FollowQTraj(q_traj_rev, t_traj, arm=arm, blocking=blocking_mode)
      PrintRobotState(ct.robot, 'p30')

