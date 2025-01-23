#!/usr/bin/python3
from core_tool import *
def Help():
  return '''Run a joystick controller.
  Usage: j'''
def Run(ct,*args):
  if ct.robot.Is('Baxter'):       ct.Run('keyctrl3')
  #elif ct.robot.Is('Motoman'):    ct.Run('keyctrl3')
  elif ct.robot.Is('Mikata'):     ct.Run('keyctrl3')
  elif ct.robot.Is('UR'):         ct.Run('keyctrl3')
  elif ct.robot.Is('Gen3'):       ct.Run('keyctrl3')
  elif ct.robot.Is('RobotiqNB'):  ct.Run('keyctrlrq')
  elif ct.robot.Is('DxlGripper'):  ct.Run('keyctrlrq')
  elif ct.robot.Is('RHP12RNGripper'):  ct.Run('keyctrlrq')
  elif ct.robot.Is('EZGripper'):  ct.Run('keyctrlrq')
  elif ct.robot.Is('DxlpO2Gripper'):  ct.Run('keyctrlrq')
  elif ct.robot.Is('DxlO3Gripper'):  ct.Run('keyctrlrq')
  else:
    CPrint(4,'Joystick-control selector is not defined for:',ct.robot.Name)
