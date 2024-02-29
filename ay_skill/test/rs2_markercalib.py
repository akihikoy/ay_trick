#!/usr/bin/python
from core_tool import *
roslib.load_manifest('sensor_msgs')
import sensor_msgs.msg

def Help():
  return '''RealSense2 camera pose calibration tool with ArUco markers.
  Usage:  rs2_markercalib ...
  '''

def Run(ct,*args):
  #rvec,tvec= [1.67829955, 1.65574508, 0.96456194], [-0.1,-0.068,0.955]
  #rvec,tvec= [2.1369, 2.2146, 0.3673], [-0.27,-0.036,0.885]
  rvec,tvec= [0.0285, 3.130955, 0.1085321], [0.08,-0.087,0.848]
  #rvec,tvec= [0, 0, 0], [-0.1,-0.068,0.955]

  Q_base= RotToQ(ExyzToRot([1,0,0],[0,1,0],[0,0,1]))
  #Q_base= RotToQ(ExyzToRot([0,0,-1],[1,0,0],[0,-1,0]))
  x= list(tvec) + list(MultiplyQ(RotToQ(Rodrigues(rvec)),Q_base))

  frame= 'camera_color_optical_frame'
  ct.viz.rs2_markercalib= TSimpleVisualizerArray(rospy.Duration(), name_space='visualizer_test', frame=frame)
  viz= ct.viz.rs2_markercalib
  viz.DeleteAllMarkers()
  viz.Reset()
  mid= 0
  alpha= 0.8
  mid= viz.AddCoord(x=x, scale=[0.1,0.02], alpha=alpha, mid=mid)

  viz.Publish()

