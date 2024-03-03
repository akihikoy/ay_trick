#!/usr/bin/python
from core_tool import *
roslib.load_manifest('sensor_msgs')
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import cv2

def Help():
  return '''RealSense2 camera pose calibration tool with ArUco markers.
  Usage:  rs2_markercalib ...
  '''

def VizMarker(ct, x_marker):
  viz= ct.viz.rs2_markercalib

  mid= 0
  alpha= 0.8
  mid= viz.AddCoord(x_marker, scale=[0.1,0.01], alpha=alpha, mid=mid)

  viz.Publish()

def ImageCallback(ct, msg, fmt):
  img= CvBridge().imgmsg_to_cv2(msg, fmt)
  if fmt=='16UC1':
    img_viz= cv2.cvtColor((img).astype('uint8'), cv2.COLOR_GRAY2BGR)
    #print np.min(img),np.max(img),'-->',np.min(img_viz),np.max(img_viz)
  else:  img_viz= img

  if msg.header.seq%2!=0:  return  #Slip for speed up

  frame= img_viz
  print 'frame=',frame.shape
  print 'dtype=',frame.dtype

  corners, ids, rejectedImgPoints= cv2.aruco.detectMarkers(frame, ct.GetAttr(TMP,'aruco','board').dictionary, parameters=ct.GetAttr(TMP,'aruco','parameters'))
  if ids is not None and len(ids)>0:
    #print 'corners:', corners
    P,K,D,R= ct.GetAttr(TMP,'cam_info')
    retval, rvec, tvec= cv2.aruco.estimatePoseBoard(corners, ids, ct.GetAttr(TMP,'aruco','board'), P, D)
    print 'retval=', retval
    print 'rvec=', rvec
    print 'tvec=', tvec

    #Convert to [x,y,z,quaternion] form:
    Q_base= RotToQ(ExyzToRot([1,0,0],[0,1,0],[0,0,1]))
    #Q_base= RotToQ(ExyzToRot([0,0,-1],[1,0,0],[0,-1,0]))
    x_marker= list(tvec) + list(MultiplyQ(RotToQ(Rodrigues(rvec)),Q_base))

    #Draw the detection on an image:
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    #cv2.drawFrameAxes(frame, P, D, rvec, tvec, length=0.05)  #For OpenCV 3.4+
    cv2.aruco.drawAxis(frame, P, D, rvec, tvec, length=0.05);

    #Visualize the detection with RViz:
    VizMarker(ct, x_marker)

  ct.SetAttr(TMP,'rs_image', frame)


def Run(ct,*args):
  topic= args[0] if len(args)>0 else '/camera/color/image_raw'
  fmt= args[1] if len(args)>1 else None
  if fmt is None:
    fmt= GetImageEncoding(topic, convert_cv=True)

  '''
  #rvec,tvec= [1.67829955, 1.65574508, 0.96456194], [-0.1,-0.068,0.955]
  #rvec,tvec= [2.1369, 2.2146, 0.3673], [-0.27,-0.036,0.885]
  rvec,tvec= [0.0285, 3.130955, 0.1085321], [0.08,-0.087,0.848]
  #rvec,tvec= [0, 0, 0], [-0.1,-0.068,0.955]

  Q_base= RotToQ(ExyzToRot([1,0,0],[0,1,0],[0,0,1]))
  #Q_base= RotToQ(ExyzToRot([0,0,-1],[1,0,0],[0,-1,0]))
  x= list(tvec) + list(MultiplyQ(RotToQ(Rodrigues(rvec)),Q_base))
  '''

  dictionary= cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
  parameters= cv2.aruco.DetectorParameters_create()
  #NOTE: Use ay_test/opencv/cpp/sample/marker/markers_1_9.svg as the board.
  board= cv2.aruco.GridBoard_create(markersX=3, markersY=3, markerLength=0.04, markerSeparation=0.02, dictionary=dictionary, firstMarker=1)
  ct.SetAttr(TMP,'aruco','parameters', parameters)
  ct.SetAttr(TMP,'aruco','board', board)

  GetCameraInfo()
  P,K,D,R= GetCameraInfo()
  P= P[:3,:3]
  ct.SetAttr(TMP,'cam_info', (P,K,D,R))

  frame= 'camera_color_optical_frame'
  ct.viz.rs2_markercalib= TSimpleVisualizerArray(rospy.Duration(), name_space='visualizer_test', frame=frame)
  viz= ct.viz.rs2_markercalib
  viz.DeleteAllMarkers()
  viz.Reset()

  ct.SetAttr(TMP,'rs_image', None)
  ct.AddSub('rs_image', topic, sensor_msgs.msg.Image, lambda msg:ImageCallback(ct,msg,fmt))

  rate_adjuster= rospy.Rate(20)
  try:
    while not rospy.is_shutdown():
      if ct.GetAttr(TMP,'rs_image') is not None:
        cv2.imshow('marker_detection',ct.GetAttr(TMP,'rs_image'))
      key= cv2.waitKey(1)&0xFF
      if key==ord('q'):
        break
      rate_adjuster.sleep()

  finally:
    ct.DelSub('rs_image')
    cv2.destroyAllWindows()



