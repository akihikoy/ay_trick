#!/usr/bin/python3
from core_tool import *
roslib.load_manifest('sensor_msgs')
import sensor_msgs.msg
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

#NOTE This config is needed to remove a warning: (marker_detection:11998): WARNING **: xxxxx: AT-SPI: Could not obtain desktop path or name
os.environ['NO_AT_BRIDGE'] = '1'

def Help():
  return '''RealSense2 camera pose calibration tool with ArUco markers.
  Usage:  test.rs2_markercalib [RS_NAME [, FMT]]
    RS_NAME: RealSense camera name in ROS.
    FMT: Image encoding format.

  Note: Test with ROSbag
  '''


#def getBoardPoints():
  #lw_Q_marker = RotToQ(ExyzToRot([-1,0,0],[0,0,1],[0,1,0]))
  #data = [0, 0.04, 0.06, 0.10, 0.12, 0.16]
  #board = []
  #for x in data:
    #for y in data:
      #point =  [0.080 - x, 0.039, 0.036 + y] + list(lw_Q_marker)
      #board.append(point)

  #return board

def VizObjPoints(ct, viz, points, rgb, scale=0.1):
  mid = 0
  alpha=0.8
  
  mid= viz.AddPoints(points=points, scale=[scale, scale], rgb=rgb, alpha=alpha, mid=mid)
  viz.Publish()

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

  print('##OptimizeRSPose##')
  #print 'sample_list [(x_marker_robot,x_marker_rs)]:'
  #for (x_marker_robot,x_marker_rs) in sample_list:  print ' ',(x_marker_robot,x_marker_rs)
  # Minimize the pose_error
  xmin,xmax= [-5,-5,-5, -5,-5,-5],[5,5,5, 5,5,5]
  tol= 1.0e-6
  print('Optimizing...')
  res= scipy.optimize.differential_evolution(pose_error, np.array([xmin,xmax]).T, strategy='best1bin', maxiter=300, popsize=20, tol=tol, mutation=(0.5, 1), recombination=0.7)
  print('')
  print('Optimization result:\n',res)
  x_rs= rs_pose_to_x(res.x)
  return x_rs

def OptimizeRSPoseWithFixedQ(ct, sample_list, q_fixed):
  import scipy.optimize
  def rs_pose_to_x(pose_rs):
    x_rs= list(pose_rs[:3]) + list(q_fixed)
    return x_rs
  def loss_diff_x(diff_x):
    pos_err= np.linalg.norm(diff_x[:3])**2
    rot_err= 0.0
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

  print('##OptimizeRSPoseWithFixedQ##')
  #print 'sample_list [(x_marker_robot,x_marker_rs)]:'
  #for (x_marker_robot,x_marker_rs) in sample_list:  print ' ',(x_marker_robot,x_marker_rs)
  # Minimize the pose_error
  xmin,xmax= [-5,-5,-5],[5,5,5]
  tol= 1.0e-6
  print('Optimizing...')
  res= scipy.optimize.differential_evolution(pose_error, np.array([xmin,xmax]).T, strategy='best1bin', maxiter=300, popsize=20, tol=tol, mutation=(0.5, 1), recombination=0.7)
  print('')
  print('Optimization result:\n',res)
  x_rs= rs_pose_to_x(res.x)
  return x_rs

# Align the axis orientation to the robot standard
# this function use only for opencv 4.x 
def rotateAxis(rvec):
  R, _ = cv2.Rodrigues(rvec)
  Rx, _ = cv2.Rodrigues(np.array([-np.pi, 0, 0]))
  newR = np.dot(R, Rx)
  r, _ = cv2.Rodrigues(newR)

  return r

def getPoints(obj_points, tvec, rvec):
  points = []
  R, _ = cv2.Rodrigues(rvec)
  for pt in obj_points:
    new_pt = np.dot(R, pt[0]) + tvec - np.dot(R, np.array([0, 0.16, 0]))
    points.append(new_pt)
  
  return points

def ImageCallback(ct, msg, fmt, rs_name):
  img= CvBridge().imgmsg_to_cv2(msg, fmt)
  if fmt=='16UC1':
    img_viz= cv2.cvtColor((img).astype('uint8'), cv2.COLOR_GRAY2BGR)
    #print np.min(img),np.max(img),'-->',np.min(img_viz),np.max(img_viz)
  else:  img_viz= img

  if msg.header.seq%2!=0:  return  #Slip for speed up

  frame= img_viz
  #print 'frame=',frame.shape
  #print 'dtype=',frame.dtype

  detector = ct.GetAttr(TMP,'aruco','detector')
  board = ct.GetAttr(TMP,'aruco','board')
  corners, ids, rejectedImgPoints = detector.detectMarkers(img)
  if ids is not None and len(ids)>0:
    #print 'corners:', corners
    P,K,D,R= ct.GetAttr(TMP,'cam_info')
    object_points, image_points = board.matchImagePoints(
        detectedCorners=corners, detectedIds=ids)
    if object_points is not None:
      # Solve for the rotation and translation vectors
      success, rvec, tvec = cv2.solvePnP(
        object_points, image_points, P, D)
      if success:
        #Rotate to align with the older version of aruco board.
        rvec = rotateAxis(rvec)

        #Convert to [x,y,z,quaternion] form:
        tvec,rvec= tvec.ravel(), rvec.ravel()
        rot= Rodrigues(rvec)
        #Shift to match with the older version of aruco board.
        marker_length= board.getMarkerLength()
        marker_separation= board.getMarkerSeparation()
        markers_x,markers_y= board.getGridSize()
        ex,ey,ez= RotToExyz(rot)
        tvec= (np.array(tvec) - (marker_length*markers_y+marker_separation*(markers_y-1))*np.array(ey))
        x_marker_rs= list(tvec) + list(RotToQ(rot))
        #print(f'tvec={tvec}, rvec={rvec}')
        
        frame= cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        frame= cv2.drawFrameAxes(frame, P, D, rvec, tvec, 0.05)

        #Visualize the detection with RViz:
        VizMarker(ct, ct.viz.rs2_markercalib_rs, x_marker_rs, scale=[0.1,0.002])
        #points = getPoints(object_points, tvec, rvec)
        #VizObjPoints(ct, ct.viz.rs2_markerpoint, points, rgb=[1,0,0],scale=0.005)

  ct.SetAttr(TMP,'rs_image', frame)

  if np.max(np.abs((ct.robot.Q())))>1e-6:
    #lw_x_marker: Marker pose in the wrist frame from a CAD model.
    lw_x_marker= ct.GetAttr(TMP,'lw_x_marker')

    x_marker_robot= ct.robot.FK(x_ext=lw_x_marker)
    #board_points = getBoardPoints()
    #points = []
    #for pt in board_points:
      #point = ct.robot.FK(x_ext = pt)
      #points.append([point[0], point[1], point[2]])
    #VizObjPoints(ct, ct.viz.rs2_markerpoint_robot, points, rgb=[0,0,1], scale=0.005)
    #Visualize the marker pose estimation from the robot-marker model.
    VizMarker(ct, ct.viz.rs2_markercalib_robot, x_marker_robot, scale=[0.1,0.001])

    #If there is a request of sampling, two marker pose estimations from RS and the robot frame are stored.
    if ct.GetAttr(TMP,'rs_sample_req'):
      ct.SetAttr(TMP,'rs_sample_req', False)
      print(x_marker_robot, x_marker_rs)
      ct.GetAttr(TMP,'rs_sample_list').append((x_marker_robot, x_marker_rs))
      print('Updated the sample list.')
      print('sample_list [(x_marker_robot,x_marker_rs)]:')
      print('[')
      for (x_marker_robot,x_marker_rs) in ct.GetAttr(TMP,'rs_sample_list'):
        print('  {},'.format([x_marker_robot,x_marker_rs]))
      print(']')

  if ct.GetAttr(TMP,'rs_optimization_req'):
    ct.SetAttr(TMP,'rs_optimization_req', False)
    #Executing the optimization to obtain the RS pose in the robot frame.
    x_rs= OptimizeRSPose(ct, ct.GetAttr(TMP,'rs_sample_list'))
    print('Optimization completed.')
    print('  x_rs=',x_rs)
    ##TEST: Optimization with fixed Q:
    #q_fixed= [0.7070980597795835, -0.7068790570623428, -0.01794432184991827, 0.003511958990169429]
    #x_rs= OptimizeRSPoseWithFixedQ(ct, ct.GetAttr(TMP,'rs_sample_list'), q_fixed)
    #print 'Optimization completed.'
    #print '  x_rs=',x_rs

  if ct.GetAttr(TMP,'rs_print_req'):
    ct.SetAttr(TMP,'rs_print_req', False)
    x_cam= TfOnce(ct.robot.BaseFrame, f'{rs_name}_color_optical_frame')
    print('x_marker_robot=',x_marker_robot)
    print('x_marker_rs=',Transform(x_cam, x_marker_rs))


def Run(ct,*args):
  rs_name= args[0] if len(args)>0 else 'camera'
  fmt= args[1] if len(args)>1 else None

  topic= f'/{rs_name}/color/image_raw'
  cam_info_topic= f'/{rs_name}/color/camera_info'
  if fmt is None:
    fmt= GetImageEncoding(topic, convert_cv=True)
  print(f'''rs2_markercalib information:
    RealSense name: {rs_name}
    RGB image topic: {topic}
    Camera info topic: {cam_info_topic}
    Image encoding: {fmt}''')

  if topic is None or cam_info_topic is None:
    raise Exception(f'topic or cam_info_topic is empty.')

  #lw_x_marker: Marker pose in the wrist frame from a CAD model.
  if not ct.HasAttr(TMP,'lw_x_marker') and ct.robot.Is('UR'):
    #  0.018: From the wrist plane to the base point of RHP12RNGripper.
    lw_Q_marker= RotToQ(ExyzToRot([-1,0,0],[0,0,1],[0,1,0]))
    lw_x_marker= [0.080, 0.039, 0.196] + list(lw_Q_marker)
    ct.SetAttr(TMP,'lw_x_marker', lw_x_marker)
  elif not ct.HasAttr(TMP,'lw_x_marker') and ct.robot.Is('Motoman'):
    #  0.034: From the wrist plane to the base point of RHP12RNGripper.
    lw_Q_marker= RotToQ(ExyzToRot([0,1,0],[0,0,1],[1,0,0]))
    lw_x_marker= [0.039, -0.080, 0.018+0.034] + list(lw_Q_marker)
    ct.SetAttr(TMP,'lw_x_marker', lw_x_marker)
  if not ct.HasAttr(TMP,'lw_x_marker'):
    raise Exception('Attribute TMP:lw_x_marker is not defined.')
  
  ## set offset of marker axis position for opencv 4.x
  #lw_Q_marker= RotToQ(ExyzToRot([-1,0,0],[0,0,1],[0,1,0]))
  #lw_x_marker= [0.080, 0.039, 0.196] + list(lw_Q_marker)
  #ct.SetAttr(TMP, 'lw_x_marker', lw_x_marker)

  ##(TEST)25deg angled RH-P12-RN with angled marker fixture.
  #lw_Q_marker= MultiplyQ(QFromAxisAngle([0,1,0],-25./180.*np.pi),RotToQ(ExyzToRot([0,-1,0],[-1,0,0],[0,0,-1]))).tolist()
  #lw_x_marker= [-0.007484, 0.07970, 0.033723] + list(lw_Q_marker)

  dictionary= cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
  parameters= cv2.aruco.DetectorParameters()
  #NOTE: Use ay_test/opencv/cpp/sample/marker/markers_1_9.svg as the board.
  board = cv2.aruco.GridBoard(size=(3,3), markerLength=0.04, markerSeparation=0.02, dictionary=dictionary, ids=np.array([1,2,3,4,5,6,7,8,9]))
  detector = cv2.aruco.ArucoDetector(dictionary, parameters)
  ct.SetAttr(TMP,'aruco','parameters', parameters)
  ct.SetAttr(TMP,'aruco','board', board)
  ct.SetAttr(TMP,'aruco','detector', detector)

  P,K,D,R= GetCameraInfo(cam_info_topic=cam_info_topic)
  P= P[:3,:3]
  ct.SetAttr(TMP,'cam_info', (P,K,D,R))
  
  ct.SetAttr(TMP,'rs_sample_req', False)
  ct.SetAttr(TMP,'rs_optimization_req', False)
  ct.SetAttr(TMP,'rs_print_req', False)

  if ct.HasAttr(TMP,'rs_sample_list') and len(ct.GetAttr(TMP,'rs_sample_list'))>0:
    print('Previous calibration data found. Do you want to continue from that?')
    print('  # of samples:',len(ct.GetAttr(TMP,'rs_sample_list')))
    if AskYesNo():
      print('Keeping the sample list')
    else:
      print('Resetting the sample list')
      ct.SetAttr(TMP,'rs_sample_list', [])
  else:
    ct.SetAttr(TMP,'rs_sample_list', [])

  frame= f'{rs_name}_color_optical_frame'

  ct.viz.rs2_markerpoint= TSimpleVisualizerArray(rospy.Duration(), name_space='viz_rs2_markerpoint', frame=frame)
  ct.viz.rs2_markercalib_rs= TSimpleVisualizerArray(rospy.Duration(), name_space='viz_rs2_markercalib_rs', frame=frame)
  ct.viz.rs2_markerpoint_robot= TSimpleVisualizerArray(rospy.Duration(), name_space='viz_rs2_markerpoint_robot', frame=ct.robot.BaseFrame)
  ct.viz.rs2_markercalib_robot= TSimpleVisualizerArray(rospy.Duration(), name_space='viz_rs2_markercalib_robot', frame=ct.robot.BaseFrame)
  for viz in (ct.viz.rs2_markercalib_rs, ct.viz.rs2_markercalib_robot, ct.viz.rs2_markerpoint, ct.viz.rs2_markerpoint_robot):
    viz.DeleteAllMarkers()
    viz.Reset()

  ct.SetAttr(TMP,'rs_image', None)
  ct.AddSub('rs_image', topic, sensor_msgs.msg.Image, lambda msg:ImageCallback(ct,msg,fmt,rs_name))

  try:
    print('''Keyboard operation:
    - q: quit.
    - space: add the current observation to the sample.
    - o: run the optimization.
    - a: add the current observation to the sample and run the optimization.
    - p: print the current observation.
    ''')
    rate_adjuster= rospy.Rate(20)
    while not rospy.is_shutdown():
      if ct.GetAttr(TMP,'rs_image') is not None:
        cv2.imshow('marker_detection',ct.GetAttr(TMP,'rs_image'))
      key= cv2.waitKey(1)&0xFF
      if key==ord('q'):
        break
      elif key==ord(' '):
        ct.SetAttr(TMP,'rs_sample_req', True)
      elif key==ord('a'):
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



