#!/usr/bin/python3
from core_tool import *
import rospkg
sys.path.append(os.path.join(rospkg.RosPack().get_path('ay_util'),'scripts'))
from proc_manager_ur import TURManager, ur_dashboard_msgs, ur_msgs

def Help():
  return '''UR manager utility.  Provide an interface to the dashboard.
    ur.ur_mngr 'setup'
    ur.ur_mngr 'on'
      Setup the UR manager.
      After setup, TURManager object can be accessed by:
      ct.GetAttr(TMP,'ur_mngr')
    ur.ur_mngr 'clear'
    ur.ur_mngr 'off'
      Disconnect from the dashboard.
    ur.ur_mngr 'is_prepared'
      Return if the UR manager is ready to use.
    ur.ur_mngr 'is_protective_stop'
      Return if the robot state is protective stop.
    ur.ur_mngr 'recover_from_protective_stop'
      Recover from protective stop, and start running the (external control) program.
  '''

def IsPrepared(ct):
  return ct.HasAttr(TMP,'ur_mngr') and isinstance(ct.GetAttr(TMP,'ur_mngr'),TURManager) and bool(ct.GetAttr(TMP,'ur_mngr').srvp_ur_dashboard)

def Run(ct,*args):
  if len(args)==0:
    command= 'clear'
  else:
    command= args[0]
    args= args[1:]

  if command in ('on','setup'):
    if ct.robot.Is('sim'):  return
    if IsPrepared(ct):  return
    ur_mngr= TURManager()
    ur_mngr.ConnectToURDashboard()
    ct.SetAttr(TMP,'ur_mngr', ur_mngr)

  elif command in ('off','clear'):
    if ct.robot.Is('sim'):  return
    if not ct.HasAttr(TMP,'ur_mngr'):
      raise Exception('Not connected to ur_mngr')
    ct.GetAttr(TMP,'ur_mngr').DisconnectUR()
    ct.DelAttr(TMP,'ur_mngr')

  elif command=='is_prepared':
    if ct.robot.Is('sim'):  return False
    return IsPrepared(ct)

  elif command=='is_protective_stop':
    if ct.robot.Is('sim'):  return False
    if not ct.HasAttr(TMP,'ur_mngr'):
      raise Exception('Not connected to ur_mngr')
    return ct.GetAttr(TMP,'ur_mngr').ur_safety_mode==ur_dashboard_msgs.msg.SafetyMode.PROTECTIVE_STOP

  elif command=='is_robot_emergency_stop':
    if ct.robot.Is('sim'):  return False
    if not ct.HasAttr(TMP,'ur_mngr'):
      raise Exception('Not connected to ur_mngr')
    return ct.GetAttr(TMP,'ur_mngr').ur_safety_mode==ur_dashboard_msgs.msg.SafetyMode.ROBOT_EMERGENCY_STOP

  elif command=='is_power_off':
    if ct.robot.Is('sim'):  return False
    if not ct.HasAttr(TMP,'ur_mngr'):
      raise Exception('Not connected to ur_mngr')
    return ct.GetAttr(TMP,'ur_mngr').ur_safety_mode==ur_dashboard_msgs.msg.SafetyMode.NORMAL and ct.GetAttr(TMP,'ur_mngr').ur_robot_mode==ur_dashboard_msgs.msg.RobotMode.POWER_OFF

  elif command=='recover_from_protective_stop':
    if ct.robot.Is('sim'):  return
    if not ct.HasAttr(TMP,'ur_mngr'):
      raise Exception('Not connected to ur_mngr')
    ur_mngr= ct.GetAttr(TMP,'ur_mngr')
    ur_mngr.WaitForProgramRunning(False)
    #print 'debug,1'
    #print 'UR.ur_safety_mode',ur_mngr.ur_safety_mode_names[ur_mngr.ur_safety_mode]
    #rospy.sleep(1.0)
    for i in range(20):
      ur_mngr.RunURDashboard('unlock_protective_stop')
      print('debug,2',i)
      print('UR.ur_safety_mode',ur_mngr.ur_safety_mode_names[ur_mngr.ur_safety_mode])
      #rospy.sleep(0.2)
      print('debug,3',i)
      print('UR.ur_safety_mode',ur_mngr.ur_safety_mode_names[ur_mngr.ur_safety_mode])
      if ur_mngr.WaitForSafetyMode(ur_dashboard_msgs.msg.SafetyMode.NORMAL, timeout=0.5):  break
    #print 'debug,4'
    #print 'UR.ur_safety_mode',ur_mngr.ur_safety_mode_names[ur_mngr.ur_safety_mode]
    rospy.sleep(0.2)
    ur_mngr.RunURDashboard('play')
    #rospy.sleep(0.2)
    ur_mngr.WaitForProgramRunning(True)

  elif command=='power_on':
    if ct.robot.Is('sim'):  return
    if not ct.HasAttr(TMP,'ur_mngr'):
      raise Exception('Not connected to ur_mngr')
    ur_mngr= ct.GetAttr(TMP,'ur_mngr')
    if ur_mngr.ur_safety_mode!=ur_dashboard_msgs.msg.SafetyMode.NORMAL or ur_mngr.ur_robot_mode!=ur_dashboard_msgs.msg.RobotMode.POWER_OFF:
      raise Exception('UR mode is not POWER_OFF.')
    ur_mngr.WaitForRobotMode(ur_dashboard_msgs.msg.RobotMode.POWER_OFF)
    ur_mngr.RunURDashboard('power_on')
    ur_mngr.WaitForRobotMode(ur_dashboard_msgs.msg.RobotMode.IDLE)
    ur_mngr.RunURDashboard('brake_release')
    ur_mngr.WaitForRobotMode(ur_dashboard_msgs.msg.RobotMode.RUNNING)
    rospy.sleep(0.2)
    ur_mngr.RunURDashboard('play')
    ur_mngr.WaitForProgramRunning(True)

  else:
    raise Exception('Unknown command:',command)
