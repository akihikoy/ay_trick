#!/usr/bin/python
from core_tool import *
def Help():
  return '''Repeating pick and place demo.
  Usage:
    fv.repeat_p_p_demo 'on' [, ARM]
      Turn on a repeat_p_p_demo thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.repeat_p_p_demo 'off' [, ARM]
      Stop a repeat_p_p_demo thread.
      ARM: RIGHT or LEFT. Default: ct.robot.Arm
    fv.repeat_p_p_demo
    fv.repeat_p_p_demo 'clear'
      Stop all threads. Equivalent to:
        fv.repeat_p_p_demo 'off' LEFT
        fv.repeat_p_p_demo 'off' RIGHT'''

def RepeatPPLoop(th_info, ct, arm):
  ct.Run('fv.ctrl_params')

  arm= 0
  lw_xf= ct.GetAttr('wrist_r','lx')
  xf_grasp= [0.42865839674738626, -0.17402002863851962, 0.042897092879979254, -0.015936991370258703, 0.6507556876174314, 0.025927674434131392, 0.758677008375808]
  xf_gup0= xf_grasp+np.array([0,0,0.03, 0,0,0,0])
  xf_gup= xf_grasp+np.array([0,0,0.10, 0,0,0,0])
  g_pregrasp= 0.06
  g_grasp= 0.034

  ct.robot.MoveGripper(g_pregrasp, arm=arm, blocking=True)
  try:
    ct.robot.MoveToX(xf_gup, x_ext=lw_xf, dt=4.0, arm=arm, blocking=True)
  except ROSError as e:
    CPrint(4,'Error: {}; tentatively ignored.'.format(e))
  rospy.sleep(0.3)

  while th_info.IsRunning() and not rospy.is_shutdown():
    try:
      ct.robot.MoveToX(xf_grasp, x_ext=lw_xf, dt=3.0, arm=arm, blocking=True)
    except ROSError as e:
      CPrint(4,'Error: {}; tentatively ignored.'.format(e))
    if not th_info.IsRunning():  break
    ct.robot.MoveGripper(g_grasp, arm=arm, blocking=True)
    rospy.sleep(0.5)
    
    #ct.Run('fv.pickup2b', 'once', arm)
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

    if not th_info.IsRunning():  break
    #for i in range(20):
      #if not th_info.IsRunning():  break
      #if ct.robot.actc.traj.get_state()==3:  break
      #print 'Robot is not in normal state.'
      #rospy.sleep(0.25)
    #if ct.robot.actc.traj.get_state()!=3:  return
    
    try:
      ct.robot.MoveToX(xf_gup0, x_ext=lw_xf, dt=3.0, arm=arm, blocking=True)
    except ROSError as e:
      CPrint(4,'Error: {}; tentatively ignored.'.format(e))
    ct.Run('fv.openif','on',arm)
    try:
      ct.robot.MoveToX(xf_grasp, x_ext=lw_xf, dt=3.0, arm=arm, blocking=True)
    except ROSError as e:
      CPrint(4,'Error: {}; tentatively ignored.'.format(e))
    rospy.sleep(0.5)
    ct.robot.MoveGripper(g_pregrasp, arm=arm, blocking=True)
    if not th_info.IsRunning():  break
    try:
      ct.robot.MoveToX(xf_gup, x_ext=lw_xf, dt=4.0, arm=arm, blocking=True)
    except ROSError as e:
      CPrint(4,'Error: {}; tentatively ignored.'.format(e))
    rospy.sleep(0.5)


def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command=='on':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    if 'repeat_p_p_demo'+LRToStrS(arm) in ct.thread_manager.thread_list:
      print 'repeat_p_p_demo'+LRToStrS(arm),'is already on'

    if not all(ct.Run('fv.fv','is_active',arm)):
      ct.Run('fv.fv','on',arm)

    CPrint(1,'Turn on:','repeat_p_p_demo'+LRToStrS(arm))
    ct.thread_manager.Add(name='repeat_p_p_demo'+LRToStrS(arm), target=lambda th_info: RepeatPPLoop(th_info,ct,arm))

  elif command=='off':
    arm= args[0] if len(args)>0 else ct.robot.Arm
    CPrint(2,'Turn off:','repeat_p_p_demo'+LRToStrS(arm))
    ct.thread_manager.Stop(name='repeat_p_p_demo'+LRToStrS(arm))

  elif command=='clear':
    CPrint(2,'Turn off:','repeat_p_p_demo'+LRToStrS(RIGHT))
    CPrint(2,'Turn off:','repeat_p_p_demo'+LRToStrS(LEFT))
    ct.thread_manager.Stop(name='repeat_p_p_demo'+LRToStrS(RIGHT))
    ct.thread_manager.Stop(name='repeat_p_p_demo'+LRToStrS(LEFT))


