#!/usr/bin/python
from core_tool import *
def Help():
  return '''Set parameters for controllers with FingerVision.
  Usage: template'''
def Run(ct,*args):
  #Designed for Baxter+Robotiq.
  ct.SetAttr('fv_ctrl','min_gstep', [0.0005,0.0005])
  ct.SetAttr('fv_ctrl','effort', [1.0,1.0])
  ct.SetAttr('fv_ctrl','tracko_gain', [[0.5,0.5,0.5]]*2)
  #ct.SetAttr('fv_ctrl','pickup2a_kp', [1.0,1.0, 15.0,  1.0,1.0,1.0])
  #ct.SetAttr('fv_ctrl','pickup2a_kd', [0.1,0.1, 2.5,  0.1,0.1,0.1])
  ct.SetAttr('fv_ctrl','pickup2a_kp', [1.0,1.0, 5.0,  1.0,1.0,1.0])
  ct.SetAttr('fv_ctrl','pickup2a_kd', [0.1,0.1, 1.0,  0.1,0.1,0.1])
  ct.SetAttr('fv_ctrl','pickup2a_lowgain', 0.3)
  ct.SetAttr('fv_ctrl','pickup2a_gtimeout1', 50)
  ct.SetAttr('fv_ctrl','pickup2a_gtimeout2', 250)
  for arm in (RIGHT,LEFT):
    if ct.robot.EndEff(arm).Is('BaxterEPG'):
      ct.GetAttr('fv_ctrl','min_gstep')[arm]= 0.002
    elif ct.robot.EndEff(arm).Is('DxlGripper'):
      ct.GetAttr('fv_ctrl','min_gstep')[arm]= 0.002
      ct.GetAttr('fv_ctrl','effort')[arm]= 100.0
  if ct.robot.Is('Mikata'):
    ct.SetAttr('fv_ctrl','tracko_gain', [[0.1,0.1,0.1]])
    ct.SetAttr('fv_ctrl','pickup2a_kp', [1.0,1.0, 5.0,  1.0,1.0,1.0])
    ct.SetAttr('fv_ctrl','pickup2a_kd', [0.1,0.1, 1.0,  0.1,0.1,0.1])
    #ct.SetAttr('fv_ctrl','pickup2a_lowgain', 0.9)
    ct.SetAttr('fv_ctrl','pickup2a_gtimeout1', 20)  #Ctrl step=50Hz(Mikata), 500Hz(Bx)
    ct.SetAttr('fv_ctrl','pickup2a_gtimeout2', 25)