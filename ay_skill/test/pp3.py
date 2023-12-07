#!/usr/bin/python
from core_tool import *
def Help():
  return '''Example of pick and place from/to three locations.
  Setup:
    $ robot
    $ roslaunch fingervision fvp_general.launch pkg_dir:=${HOME}/data config1:=config/fvp300x_l.yaml config2:=config/fvp300x_r.yaml publish_image:=false
    > robot
    > viz ''
    > fv_names,node_names= {'A':{RIGHT:'fvp_1_r',LEFT:'fvp_1_l'}},None
    > fv.fv 'on', 'all', fv_names, node_names, 'no_wrench'
  Usage: test.pp3 [SPEED_INDEX [, CTRL_SLEEP [, WITH_FV]]]
    SPEED_INDEX: Index of speed configuration (default=10; slow; see the list).
        If SPEED_INDEX is not int, it is assumed to be (LIN_SPEED,LIN_ACC).
        LIN_SPEED: Linear speed m/s.
        LIN_ACC: Linear acceleration m/s^2.
    CTRL_SLEEP: Sleep time in seconds after each trajectory control (default=0.2).
    WITH_FV: Using FV-based gripper control (default=True).
  '''

def FollowXTraj(ct, x_traj, x_ext, q_seed, arm, lin_speed, lin_acc, tool_rad, joint_rad, idx_split):
  q_traj= ct.robot.XTrajToQTraj(x_traj, start_angles=q_seed, x_ext=x_ext, arm=arm)
  x_trajs= [x_traj[:, :3], x_traj[:, 3:], q_traj]

  if len(lin_speed)==1:
    kwargs1= dict(speed=lin_speed[0],
                speed_ratios=[1.0/tool_rad,1.0/joint_rad],
                acceleration=lin_acc[0],
                dist_criteria=['E', 'Q', 'E'])
    tx_traj= TTrajFromXTraj3(x_trajs, **kwargs1)
    print('q_traj:',len(q_traj))
    print('x_traj:',len(x_traj))
    print('tx_traj:',len(tx_traj))
  else:
    kwargs1= dict(speed=lin_speed[0],
                speed_ratios=[1.0/tool_rad,1.0/joint_rad],
                acceleration=lin_acc[0],
                dist_criteria=['E', 'Q', 'E'])
    kwargs2= dict(speed=lin_speed[1],
                speed_ratios=[1.0/tool_rad,1.0/joint_rad],
                acceleration=lin_acc[1],
                dist_criteria=['E', 'Q', 'E'])
    tx_traj_1= TTrajFromXTraj3(x_trajs, **kwargs1)
    tx_traj_2= TTrajFromXTraj3(x_trajs, **kwargs2)

    dt_traj_connection= tx_traj_1[idx_split] - tx_traj_2[idx_split]
    #dt_traj_connection+= delay
    tx_traj= tx_traj_1[:idx_split+1] + [t + dt_traj_connection for t in tx_traj_2[idx_split+1:]]

  f_fk= lambda q: ct.robot.FK(q,x_ext=x_ext,arm=arm)
  if not CheckXQTrajValidity(q_traj, x_traj, f_fk):  raise ROSError('ik-traj-check','x_traj is invalid')
  #if CheckCollisionQTraj(ct, q_traj, arm):  raise ROSError('collision-check','q_traj is in collision')

  #TEST: Replace the first point with the current joint angles (needed for Motoman):
  q_traj[0]= ct.robot.Q(arm=arm)
  ct.robot.FollowQTraj(q_traj, tx_traj, arm=arm, blocking=True)


def Run(ct,*args):
  speed_index= args[0] if len(args)>0 else 10
  ctrl_sleep= args[1] if len(args)>1 else 0.2
  with_fv= args[2] if len(args)>2 else True
  arm= 0
  #Fingertip pose (local pose in the wrist frame).
  lx_f= ct.GetAttr('wrist_'+LRToStrs(ct.robot.Arm),'lx')

  if with_fv and not ct.Run('fv.fv', 'is_active')[0]:
    with_fv= False
    CPrint(3,'Set with_fv=False as FV is not configured')

  #Common orientation:
  qf_cmn= RotToQ([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]).tolist()

  pf_pick_list= [
    [0.567, 0.15758975299434502, -0.23492058486461642],
    [0.495, 0.15758975299434502, -0.23492058486461642],
    [0.423, 0.15758975299434502, -0.23492058486461642],
    ]
  q_init= [0.6747981905937195, -0.879830002784729, -0.05734863132238388, 0.20483757555484772]

  assert(len(q_init)==ct.robot.DoF())

  g_open= 0.04
  g_pregrasp= 0.026
  #g_pregrasp= 0.03

  pf_pick_list= np.array(pf_pick_list)

  xf_grasp_list= [pf_pick.tolist()+qf_cmn for pf_pick in pf_pick_list]
  xf_pregrasp_list= [(pf_pick+[0,0,0.1]).tolist()+qf_cmn for pf_pick in pf_pick_list]
  xf_place_list= [(pf_pick+[0,-0.3,0]).tolist()+qf_cmn for pf_pick in pf_pick_list]
  xf_preplace_list= [(pf_pick+[0,-0.3,0.1]).tolist()+qf_cmn for pf_pick in pf_pick_list]

  speed_list= {
    1  : (0.1  ,0.25 ),
    4  : (0.4  ,0.5  ),
    10 : (1.0  ,0.5  ),
    20 : (2.0  ,1.0  ),
    40 : (4.0  ,2.0  ),
    60 : (6.0  ,4.0  ),
    80 : (8.0  ,4.0  ),
    100: (10.0 ,5.0  ),
    120: (12.0 ,6.0  ),
    160: (16.0 ,8.0  ),  #Max of working stably
    180: (18.0 ,9.0  ),  #Working
    200: (20.0 ,9.0  ),  #Working
    #200: (20.0 ,10.0 ),  #Error (Robot Alarm 4414: Segment over R1:HIGH SL[U]R)
    220: (22.0 ,9.0  ),  #Working
    #220: (22.0 ,12.0 ),  #Error (Driver error: Validation failed: Max velocity exceeded for trajectory pt 4, joint 'joint_3_u')
  }
  if isinstance(speed_index, int):
    lin_speed,lin_acc= speed_list[speed_index]
  else:
    lin_speed,lin_acc= speed_index
  ftargs_mv= dict(x_ext=lx_f, arm=arm, lin_speed=[lin_speed], lin_acc=[lin_acc], tool_rad=0.05, joint_rad=0.01, idx_split=None)
  ftargs_pk= dict(x_ext=lx_f, arm=arm, lin_speed=[0.04,lin_speed], lin_acc=[0.2,lin_acc], tool_rad=0.05, joint_rad=0.01, idx_split=2)
  if not with_fv:
    #Disable slow-down motion during picking up:
    ftargs_pk= ftargs_mv

  #Move to the initial pose:
  ct.robot.MoveGripper(g_open)
  ct.robot.MoveToQ(q_init, dt=4.0, blocking=True)
  rospy.sleep(ctrl_sleep)

  xf_curr= ct.robot.FK(x_ext=lx_f)

  #Pick and place the 1st item:
  xf_traj11= np.array( [xf_curr]
                      + XInterpolation(xf_curr,xf_grasp_list[0],5) )
  g_trg11= g_pregrasp

  FollowXTraj(ct, xf_traj11, q_seed=ct.robot.Q(arm=arm), **ftargs_mv)
  ct.robot.MoveGripper(g_trg11)
  rospy.sleep(ctrl_sleep)
  if with_fv:  ct.Run('fv.hold','on',arm)

  t_start= rospy.Time.now()

  xf_traj12= np.array( [xf_grasp_list[0]]
                      + XInterpolation(xf_grasp_list[0],xf_pregrasp_list[0],5)
                      + XInterpolation(xf_pregrasp_list[0],xf_preplace_list[0],5)
                      #+ [xf_preplace_list[0]]
                      + XInterpolation(xf_preplace_list[0],xf_place_list[0],5) )
  g_trg12= g_open

  FollowXTraj(ct, xf_traj12, q_seed=ct.robot.Q(arm=arm), **ftargs_pk)
  if with_fv:  ct.Run('fv.hold','off',arm)
  ct.robot.MoveGripper(g_trg12)
  rospy.sleep(ctrl_sleep)

  #Pick and place the 2nd item:
  xf_traj21= np.array( [xf_place_list[0]]
                      + XInterpolation(xf_place_list[0],xf_preplace_list[0],5)
                      + XInterpolation(xf_preplace_list[0],xf_pregrasp_list[1],5)
                      #+ [xf_pregrasp_list[1]]
                      + XInterpolation(xf_pregrasp_list[1],xf_grasp_list[1],5) )
  g_trg21= g_pregrasp

  FollowXTraj(ct, xf_traj21, q_seed=ct.robot.Q(arm=arm), **ftargs_mv)
  ct.robot.MoveGripper(g_trg21)
  rospy.sleep(ctrl_sleep)
  if with_fv:  ct.Run('fv.hold','on',arm)

  if not ct.HasAttr(TMP,'CycleTime'):  ct.SetAttr(TMP,'CycleTime', [])
  ct.GetAttr(TMP,'CycleTime').append((rospy.Time.now()-t_start).to_sec())
  print('Cycle time: {} s'.format(ct.GetAttr(TMP,'CycleTime')[-1]))
  t_start= rospy.Time.now()

  xf_traj22= np.array( [xf_grasp_list[1]]
                      + XInterpolation(xf_grasp_list[1],xf_pregrasp_list[1],5)
                      + XInterpolation(xf_pregrasp_list[1],xf_preplace_list[1],5)
                      #+ [xf_preplace_list[1]]
                      + XInterpolation(xf_preplace_list[1],xf_place_list[1],5) )
  g_trg22= g_open

  FollowXTraj(ct, xf_traj22, q_seed=ct.robot.Q(arm=arm), **ftargs_pk)
  if with_fv:  ct.Run('fv.hold','off',arm)
  ct.robot.MoveGripper(g_trg22)
  rospy.sleep(ctrl_sleep)

  #Pick and place the 3rd item:
  xf_traj31= np.array( [xf_place_list[1]]
                      + XInterpolation(xf_place_list[1],xf_preplace_list[1],5)
                      + XInterpolation(xf_preplace_list[1],xf_pregrasp_list[2],5)
                      #+ [xf_pregrasp_list[2]]
                      + XInterpolation(xf_pregrasp_list[2],xf_grasp_list[2],5) )
  g_trg31= g_pregrasp

  FollowXTraj(ct, xf_traj31, q_seed=ct.robot.Q(arm=arm), **ftargs_mv)
  ct.robot.MoveGripper(g_trg31)
  rospy.sleep(ctrl_sleep)
  if with_fv:  ct.Run('fv.hold','on',arm)

  ct.GetAttr(TMP,'CycleTime').append((rospy.Time.now()-t_start).to_sec())
  print('Cycle time: {} s'.format(ct.GetAttr(TMP,'CycleTime')[-1]))
  #t_start= rospy.Time.now()

  xf_traj32= np.array( [xf_grasp_list[2]]
                      + XInterpolation(xf_grasp_list[2],xf_pregrasp_list[2],5)
                      + XInterpolation(xf_pregrasp_list[2],xf_preplace_list[2],5)
                      #+ [xf_preplace_list[2]]
                      + XInterpolation(xf_preplace_list[2],xf_place_list[2],5) )
  g_trg32= g_open

  FollowXTraj(ct, xf_traj32, q_seed=ct.robot.Q(arm=arm), **ftargs_pk)
  if with_fv:  ct.Run('fv.hold','off',arm)
  ct.robot.MoveGripper(g_trg32)
  rospy.sleep(ctrl_sleep)

  #ct.robot.MoveToX(xf_preplace_list[2], dt=4.0, x_ext=lx_f, blocking=True)
  xf_traj_f= np.array( [xf_place_list[2]]
                      + XInterpolation(xf_place_list[2],xf_preplace_list[2],5) )
  FollowXTraj(ct, xf_traj_f, q_seed=ct.robot.Q(arm=arm), **ftargs_mv)

