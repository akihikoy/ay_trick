#!/usr/bin/python
from core_tool import *
roslib.load_manifest('sensor_msgs')
import sensor_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
import cv2

#NOTE This config is needed to remove a warning: (marker_detection:11998): WARNING **: xxxxx: AT-SPI: Could not obtain desktop path or name
os.environ['NO_AT_BRIDGE'] = '1'

def Help():
  return '''RealSense2 camera pose calibration tool with ArUco markers.
  Usage:  rs2_markercalib ...

  Note: Test with ROSbag
  '''

def VizMarker(ct, viz, x_marker, scale=[0.1,0.01]):
  mid= 0
  alpha= 0.8
  mid= viz.AddCoord(x_marker, scale=scale, alpha=alpha, mid=mid)

  viz.Publish()

def OptimizeRSPose(ct, sample_list):
  import scipy.optimize
  def rs_pose_to_x(pose_rs):
    x_rs= list(pose_rs[:3]) + list(RotToQ(Rodrigues(pose_rs[3:])))
    return x_rs
  def loss_diff_x(diff_x):
    pos_err= np.linalg.norm(diff_x[:3])**2
    rot_err= np.linalg.norm(diff_x[3:])**2
    err= 30.0*pos_err+rot_err
    #print pos_err, rot_err
    #print '{:.2e}'.format(err),
    return err
  num_f_eval= [0]
  def pose_error(pose_rs):
    x_rs= rs_pose_to_x(pose_rs)
    err= sum(loss_diff_x(DiffX(x_marker_robot,Transform(x_rs,x_marker_rs)))
             for (x_marker_robot,x_marker_rs) in sample_list)
    if num_f_eval[0]%100==0:
      sys.stderr.write(' {:.2e}'.format(err))
      sys.stderr.flush()
    num_f_eval[0]+= 1
    return err

  print '##OptimizeRSPose##'
  print 'sample_list [(x_marker_robot,x_marker_rs)]:'
  for (x_marker_robot,x_marker_rs) in sample_list:  print ' ',(x_marker_robot,x_marker_rs)
  # Minimize the pose_error
  xmin,xmax= [-5,-5,-5, -5,-5,-5],[5,5,5, 5,5,5]
  tol= 1.0e-5
  print 'Optimizing...'
  res= scipy.optimize.differential_evolution(pose_error, np.array([xmin,xmax]).T, strategy='best1bin', maxiter=300, popsize=10, tol=tol, mutation=(0.5, 1), recombination=0.7)
  print ''
  print 'Optimization result:\n',res
  x_rs= rs_pose_to_x(res.x)
  return x_rs


def ImageCallback(ct, msg, fmt):
  img= CvBridge().imgmsg_to_cv2(msg, fmt)
  if fmt=='16UC1':
    img_viz= cv2.cvtColor((img).astype('uint8'), cv2.COLOR_GRAY2BGR)
    #print np.min(img),np.max(img),'-->',np.min(img_viz),np.max(img_viz)
  else:  img_viz= img

  if msg.header.seq%2!=0:  return  #Slip for speed up

  frame= img_viz
  #print 'frame=',frame.shape
  #print 'dtype=',frame.dtype

  corners, ids, rejectedImgPoints= cv2.aruco.detectMarkers(frame, ct.GetAttr(TMP,'aruco','board').dictionary, parameters=ct.GetAttr(TMP,'aruco','parameters'))
  if ids is not None and len(ids)>0:
    #print 'corners:', corners
    P,K,D,R= ct.GetAttr(TMP,'cam_info')
    retval, rvec, tvec= cv2.aruco.estimatePoseBoard(corners, ids, ct.GetAttr(TMP,'aruco','board'), P, D)
    #print 'retval=', retval
    #print 'rvec=', rvec
    #print 'tvec=', tvec

    #Draw the detection on an image:
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    #cv2.drawFrameAxes(frame, P, D, rvec, tvec, length=0.05)  #For OpenCV 3.4+
    cv2.aruco.drawAxis(frame, P, D, rvec, tvec, length=0.05);

    #Convert to [x,y,z,quaternion] form:
    tvec,rvec= tvec.ravel(), rvec.ravel()
    x_marker_rs= list(tvec) + list(RotToQ(Rodrigues(rvec)))

    #Visualize the detection with RViz:
    VizMarker(ct, ct.viz.rs2_markercalib_rs, x_marker_rs, scale=[0.1,0.002])

  ct.SetAttr(TMP,'rs_image', frame)

  if np.max(np.abs((ct.robot.Q())))>1e-3:
    #lw_x_marker: Marker pose in the wrist frame from a CAD model.
    #  0.034: From the wrist plane to the base point of RHP12RNGripper.
    lw_Q_marker= RotToQ(ExyzToRot([0,1,0],[0,0,1],[1,0,0]))
    lw_x_marker= [0.039, -0.080, 0.018+0.034] + list(lw_Q_marker)
    x_marker_robot= ct.robot.FK(x_ext=lw_x_marker)

    #Visualize the marker pose estimation from the robot-marker model.
    VizMarker(ct, ct.viz.rs2_markercalib_robot, x_marker_robot, scale=[0.1,0.001])

    #If there is a request of sampling, two marker pose estimations from RS and the robot frame are stored.
    if ct.GetAttr(TMP,'rs_sample_req'):
      ct.SetAttr(TMP,'rs_sample_req', False)
      ct.GetAttr(TMP,'rs_sample_list').append((x_marker_robot, x_marker_rs))

  if ct.GetAttr(TMP,'rs_optimization_req'):
    ct.SetAttr(TMP,'rs_optimization_req', False)
    #Executing the optimization to obtain the RS pose in the robot frame.
    x_rs= OptimizeRSPose(ct, ct.GetAttr(TMP,'rs_sample_list'))
    print 'Optimization completed.'
    print '  x_rs=',x_rs

  if ct.GetAttr(TMP,'rs_print_req'):
    ct.SetAttr(TMP,'rs_print_req', False)
    x_cam= TfOnce(ct.robot.BaseFrame, 'camera_color_optical_frame')
    print 'x_marker_robot=',x_marker_robot
    print 'x_marker_rs=',Transform(x_cam, x_marker_rs)


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

  ct.SetAttr(TMP,'rs_sample_req', False)
  ct.SetAttr(TMP,'rs_sample_list', [])
  ct.SetAttr(TMP,'rs_optimization_req', False)
  ct.SetAttr(TMP,'rs_print_req', False)

  frame= 'camera_color_optical_frame'
  ct.viz.rs2_markercalib_rs= TSimpleVisualizerArray(rospy.Duration(), name_space='viz_rs2_markercalib_rs', frame=frame)
  ct.viz.rs2_markercalib_robot= TSimpleVisualizerArray(rospy.Duration(), name_space='viz_rs2_markercalib_robot', frame=ct.robot.BaseFrame)
  for viz in (ct.viz.rs2_markercalib_rs, ct.viz.rs2_markercalib_robot):
    viz.DeleteAllMarkers()
    viz.Reset()

  ct.SetAttr(TMP,'rs_image', None)
  ct.AddSub('rs_image', topic, sensor_msgs.msg.Image, lambda msg:ImageCallback(ct,msg,fmt))

  try:
    print '''Keyboard operation:
    - q: quit.
    - space: add the current observation to the sample (and run the optimization).
    - o: run the optimization.
    - p: print the current observation.
'''
    rate_adjuster= rospy.Rate(20)
    while not rospy.is_shutdown():
      if ct.GetAttr(TMP,'rs_image') is not None:
        cv2.imshow('marker_detection',ct.GetAttr(TMP,'rs_image'))
      key= cv2.waitKey(1)&0xFF
      if key==ord('q'):
        break
      elif key==ord(' '):
        ct.SetAttr(TMP,'rs_sample_req', True)
        ct.SetAttr(TMP,'rs_optimization_req', True)
      elif key==ord('o'):
        ct.SetAttr(TMP,'rs_optimization_req', True)
      elif key==ord('p'):
        ct.SetAttr(TMP,'rs_print_req', True)
      rate_adjuster.sleep()

  finally:
    ct.DelSub('rs_image')
    cv2.destroyAllWindows()



