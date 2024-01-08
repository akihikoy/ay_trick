#!/usr/bin/python
from core_tool import *

def Help():
  return '''Setup for Gen3+FV demo.
  Assumption: Following ROS nodes are launched beforehand.
    - Gen3 robot driver
    - FV
    Use $ rosrun ay_util ur_gui.py
  Usage: ur.setup [ROBOT_NAME [, NO_ASK]]
    ROBOT_NAME: Robot mode (default='robot_code').
    NO_ASK: If True, the system does not ask if run each command (default=False).
  '''

def Run(ct,*args):
  try:
    robot_code= rospy.get_param('robot_code')
  except KeyError:
    robot_code= 'NoRobot'
  robot_name= args[0] if len(args)>0 else robot_code
  no_ask= args[1] if len(args)>1 else False
  if robot_name is None:  robot_name= robot_code
  #fv_names,node_names= {'A':{RIGHT:'fvp_1_r',LEFT:'fvp_1_l'}},{'A':'fvp_1'}
  if not robot_name.endswith('_SIM'):
    command_list=[
      ['robot', robot_name],
      ['fv.fv', 'on', 'all', {'A':{RIGHT:'fvp_1_r',LEFT:'fvp_1_l'}}],
      ['viz',''],
      ]
  else:
    command_list=[
      ['robot', robot_name],
      ['fv.fv', 'on', 'all', {'A':{RIGHT:'fvp_1_r',LEFT:'fvp_1_l'}}],
      ['viz',''],
      ]
  for cmd in command_list:
    print ''
    CPrint(2,'Running command: > ',cmd[0],', '.join(map(repr,cmd[1:])) )
    if not no_ask:
      print '  y:Continue, s:Skip, q:Quit'
      res= KBHAskGen('y','s','q')
    else:
      res= 'y'
    if res=='y':
      ct.Run(*cmd)
    elif res=='s':
      pass
    elif res=='q':
      break
    #Bruteforce bug fix to ensure the robot is completely setup before observe.
    rate= rospy.Rate(20)
    t_start= rospy.Time.now()
    while (rospy.Time.now()-t_start).to_sec()<3.0 and not rospy.is_shutdown():
      try:
        q= ct.robot.Q()
        g= ct.robot.GripperPos()
        print 'The robot is ready!'
        break
      except Exception:
        print 'The robot is not ready...'
      rate.sleep()
