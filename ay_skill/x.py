#!/usr/bin/python3
from core_tool import *
def Help():
  return '''Display current end effector pose (x,y,z,quaternion).
  Usage: q'''
def Run(ct,*args):
  return list(ct.robot.FK())
