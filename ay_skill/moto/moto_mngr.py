#!/usr/bin/python
from core_tool import *
roslib.load_manifest('ay_util_msgs')
import ay_util_msgs.srv
import std_srvs.srv
import ur_msgs.msg

def Help():
  return '''Motoman manager utility.
    Assuming the following ROS nodes are working:
      - MotoROS (/motion_streaming_interface)
      - PUI server (ur_pui_server) with GPIO server (eth_gpio_node)
    moto.moto_mngr 'setup'
    moto.moto_mngr 'on'
      Setup the Motoman manager.
      After setup, services to enable/disable the robot and set_pui are available.
      io_states (GPIO) is embedded to ct.robot similar to the UR robot.
      Note that the set_pui service can be accessed by ct.srvp.set_pui
    moto.moto_mngr 'clear'
    moto.moto_mngr 'off'
      Disconnect from the services.
    moto.moto_mngr 'is_prepared'
      Return if the Motoman manager is ready to use.
    moto.moto_mngr 'is_ready_to_move'
      Return if the robot is ready to move.
    moto.moto_mngr 'enable'
      Enable the robot.
    moto.moto_mngr 'disable'
      Disable the robot.
  '''

def IOStates(robot):
  with robot.io_states_locker:
    io_states= robot.io_states
  return io_states

def IOStatesCallback(robot, msg):
  with robot.io_states_locker:
    robot.io_states= msg
  if robot.io_states_callback:
    robot.io_states_callback(robot.io_states)

def Connect(ct):
  ct.AddSrvP('set_pui','/ur_pui_server/set_pui',ay_util_msgs.srv.SetPUI,time_out=5.0)
  ct.AddSrvP('robot_enable','/robot_enable',std_srvs.srv.Trigger,time_out=5.0)
  ct.AddSrvP('robot_disable','/robot_disable',std_srvs.srv.Trigger,time_out=5.0)

  #Bind io_states to ct.robot.
  ct.robot.io_states= None
  ct.robot.io_states_callback= None  #User defined callback f(ur_msgs.msg.IOStates).
  ct.robot.io_states_locker= threading.RLock()
  ct.robot.IOStates= lambda robot=ct.robot: IOStates(robot)
  ct.robot.IOStatesCallback= lambda msg, robot=ct.robot: IOStatesCallback(robot,msg)
  ct.robot.AddSub('io_states', '/ur_hardware_interface/io_states', ur_msgs.msg.IOStates, ct.robot.IOStatesCallback)

def IsPrepared(ct):
  return all(srv in ct.srvp for srv in ('set_pui','robot_enable','robot_disable')) and 'io_states' in ct.robot.sub

def Disconnect(ct):
  for srv in ('set_pui','robot_enable','robot_disable'):
    ct.DelSrvP(srv)
  ct.robot.DelSub('io_states')

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command in ('on','setup'):
    if ct.robot.Is('sim'):  return
    if IsPrepared(ct):  return
    Connect(ct)

  elif command in ('off','clear'):
    if ct.robot.Is('sim'):  return
    Disconnect(ct)

  elif command=='is_prepared':
    if ct.robot.Is('sim'):  return False
    return IsPrepared(ct)

  elif command=='is_ready_to_move':
    if ct.robot.Is('sim'):  return True
    return ct.robot.IsNormal()

  elif command=='enable':
    if ct.robot.Is('sim'):  return
    if not IsPrepared(ct):  Connect(ct)
    if ct.robot.IsNormal():  return  #Do nothing if the robot is already movable.
    ct.srvp.robot_disable()  #This is necessary due to a bug.
    rospy.sleep(0.1)
    ct.srvp.robot_enable()
    rospy.sleep(0.1)

  elif command=='disable':
    if ct.robot.Is('sim'):  return
    if not IsPrepared(ct):  Connect(ct)
    ct.srvp.robot_disable()

  else:
    raise Exception('Unknown command:',command)
