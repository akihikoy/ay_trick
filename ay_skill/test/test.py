#!/usr/bin/python3
from core_tool import *
def Help():
  return '''Test.
  Usage: test [FLOAT_VALUE]'''
def Run(ct,*args):
  print(ct)
  print('Hello world!')
  if len(args)!=1:
    print(Help())
  else:
    print('args[0]=',float(args[0]))
    print('QToRot(QFromAxisAngle([1,1,1],float(args[0])))=')
    print(QToRot(QFromAxisAngle([1,1,1],float(args[0]))))
    return QToRot(QFromAxisAngle([1,1,1],float(args[0])))
