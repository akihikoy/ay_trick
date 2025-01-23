#!/usr/bin/python3
from core_tool import *
def Help():
  return '''Test of sequence of motions with linear trajectory in Cartesian.
  Usage: test.seq_motion_lin [SPEED_INDEX [, NUM_REPEAT [, CTRL_SLEEP]]]
    SPEED_INDEX: Index of speed configuration (default=10; slow; see the list).
        If SPEED_INDEX is not int, it is assumed to be (LIN_SPEED,LIN_ACC).
        LIN_SPEED: Linear speed m/s.
        LIN_ACC: Linear acceleration m/s^2.
    NUM_REPEAT: Number of repeating the cycle motions.
    CTRL_SLEEP: Sleep time in seconds after each trajectory control (default=0.0).
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
    #print('q_traj:',len(q_traj))
    #print('x_traj:',len(x_traj))
    #print('tx_traj:',len(tx_traj))
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
  num_repeat= args[1] if len(args)>1 else 1
  ctrl_sleep= args[2] if len(args)>2 else 0.0
  arm= 0
  #Fingertip pose (local pose in the wrist frame).
  if ct.HasAttr('wrist_'+LRToStrs(ct.robot.Arm),'lx'):
    lx_f= ct.GetAttr('wrist_'+LRToStrs(ct.robot.Arm),'lx')
  else:
    #In case a gripper is not recognized, a preset fingertip local pose of ThG is used.
    if ct.robot.Is('UR'):
      lx_f= [0.0,0.0,0.218, 0.5,-0.5,0.5,0.5]
    elif ct.robot.Is('MotomanGP7'):
      lx_f= [0.0,0.0,0.223638, 0.7071067811865475, 0.0, 0.7071067811865475, 0.0]
    elif ct.robot.Is('MotomanSG650'):
      lx_f= [0.0,0.0,0.21757, 0.7071067811865475, 0.0, 0.7071067811865475, 0.0]

  if ct.robot.Is('MotomanSG650'):
    #Common orientation:
    qf_cmn= RotToQ([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]).tolist()

    pf_pick_list= [
      [0.567, 0.15758975299434502, -0.23492058486461642],
      #[0.495, 0.15758975299434502, -0.23492058486461642],
      #[0.423, 0.15758975299434502, -0.23492058486461642],
      ]
    q_init= [0.6747981905937195, -0.879830002784729, -0.05734863132238388, 0.20483757555484772]

    assert(len(q_init)==ct.robot.DoF())

  elif ct.robot.Is('MotomanGP7'):
    #Common orientation:
    qf_cmn= RotToQ([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]).tolist()

    zf= 0.2
    pf_pick_list= [
      [0.400, 0.15759, zf],
      #[0.567, 0.15759, zf],
      #[0.502, 0.15759, zf],
      #[0.437, 0.15759, zf],
      ]
    #movexe pf_pick_list[0][:2]+[zf+0.1]+qf_cmn
    q_init= [0.37530166512278246, -0.039217900615927885, -0.5273505564938089, 8.4090590766276e-08, -1.0826632721858398, -0.3753017138172491]

    assert(len(q_init)==ct.robot.DoF())

  elif ct.robot.Is('MotomanHC10SDTP'):
    #Common orientation:
    qf_cmn= RotToQ([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]).tolist()

    zf= 0.0
    pf_pick_list= [
      [0.567, 0.15759, zf],
      #[0.502, 0.15759, zf],
      #[0.437, 0.15759, zf],
      ]
    #movexe pf_pick_list[0][:2]+[zf+0.1]+qf_cmn
    q_init= [-0.0077850425408454394, 0.5867028016435086, -0.1413468431913915, 5.3521207950985214e-08, -0.8427465880130912, -1.5630114247298614]

    assert(len(q_init)==ct.robot.DoF())

  else:
    raise Exception('test.seq_motion_lin: The parameters of this script are not configured for {}'.format(ct.robot.Name))

  if ct.HasAttr(TMP,'CycleTime'):
    print('A log of CycleTime exists. Do you want to remove?')
    if AskYesNo():  ct.SetAttr(TMP,'CycleTime', [])
  else:
    ct.SetAttr(TMP,'CycleTime', [])

  pf_pick_list= np.array(pf_pick_list)

  xf_grasp_list= [pf_pick.tolist()+qf_cmn for pf_pick in pf_pick_list]
  xf_pregrasp_list= [(pf_pick+[0,0,0.1]).tolist()+qf_cmn for pf_pick in pf_pick_list]
  xf_place_list= [(pf_pick+[0,-0.3,0]).tolist()+qf_cmn for pf_pick in pf_pick_list]
  xf_preplace_list= [(pf_pick+[0,-0.3,0.1]).tolist()+qf_cmn for pf_pick in pf_pick_list]

  speed_list= {
    1  : (0.1  ,0.25 ),
    2  : (0.2  ,0.5  ),  #Working,HC10SDTP@SpdLim=250mm/s
    3  : (0.3  ,0.5  ),
    4  : (0.4  ,0.5  ),
    10 : (1.0  ,0.5  ),  #Working,HC10SDTP@SpdLim=1500mm/s
    20 : (2.0  ,1.0  ),
    40 : (4.0  ,2.0  ),  #Working,HC10SDTP@SpdLim=1500mm/s
    60 : (6.0  ,4.0  ),
    80 : (8.0  ,4.0  ),
    100: (10.0 ,5.0  ),  #Working,SG650,GP7
    120: (12.0 ,6.0  ),
    160: (16.0 ,8.0  ),  #Max of working stably,SG650
    180: (18.0 ,9.0  ),  #Working,SG650
    200: (20.0 ,9.0  ),  #Working,SG650,GP7
    #200: (20.0 ,10.0 ),  #Error (Robot Alarm 4414: Segment over R1:HIGH SL[U]R)
    220: (22.0 ,9.0  ),  #Working,SG650,GP7
    #220: (22.0 ,12.0 ),  #Error (Driver error: Validation failed: Max velocity exceeded for trajectory pt 4, joint 'joint_3_u')
  }
  if isinstance(speed_index, int):
    lin_speed,lin_acc= speed_list[speed_index]
  else:
    lin_speed,lin_acc= speed_index
  ftargs_mv= dict(x_ext=lx_f, arm=arm, lin_speed=[lin_speed], lin_acc=[lin_acc], tool_rad=0.05, joint_rad=0.01, idx_split=None)

  #Move to the initial pose:
  q_diff= np.max(np.abs(np.array(q_init)-ct.robot.Q(arm=arm)))
  if q_diff>1e-5:
    ct.robot.MoveToQ(q_init, dt=max(1,q_diff/0.1), blocking=True)
    rospy.sleep(ctrl_sleep)

  xf_curr= ct.robot.FK(x_ext=lx_f)

  #Move to the first pick pose.
  xf_traj11= np.array( [xf_curr]
                      + XInterpolation(xf_curr,xf_grasp_list[0],5) )

  FollowXTraj(ct, xf_traj11, q_seed=ct.robot.Q(arm=arm), **ftargs_mv)
  rospy.sleep(ctrl_sleep)

  for i_cycle in range(num_repeat):
    t_start= rospy.Time.now()

    #Move to pick to place.
    xf_traj12= np.array( [xf_grasp_list[0]]
                        + XInterpolation(xf_grasp_list[0],xf_pregrasp_list[0],5)
                        + XInterpolation(xf_pregrasp_list[0],xf_preplace_list[0],5)
                        + XInterpolation(xf_preplace_list[0],xf_place_list[0],5) )

    FollowXTraj(ct, xf_traj12, q_seed=ct.robot.Q(arm=arm), **ftargs_mv)
    rospy.sleep(ctrl_sleep)

    #Move back from place to pick.
    xf_traj21= np.array( [xf_place_list[0]]
                        + XInterpolation(xf_place_list[0],xf_preplace_list[0],5)
                        + XInterpolation(xf_preplace_list[0],xf_pregrasp_list[0],5)
                        + XInterpolation(xf_pregrasp_list[0],xf_grasp_list[0],5) )
    FollowXTraj(ct, xf_traj21, q_seed=ct.robot.Q(arm=arm), **ftargs_mv)
    rospy.sleep(ctrl_sleep)

    ct.GetAttr(TMP,'CycleTime').append((rospy.Time.now()-t_start).to_sec())
    print(('Cycle time: {} s'.format(ct.GetAttr(TMP,'CycleTime')[-1])))
    t_start= rospy.Time.now()

  print(('Average cycle time: {} s'.format(np.mean(ct.GetAttr(TMP,'CycleTime')))))
  print(('        stdev: {} s'.format(np.std(ct.GetAttr(TMP,'CycleTime')))))

  #Move to the initial pose:
  q_diff= np.max(np.abs(np.array(q_init)-ct.robot.Q(arm=arm)))
  if q_diff>1e-5:
    ct.robot.MoveToQ(q_init, dt=max(1,q_diff/0.1), blocking=True)
    rospy.sleep(ctrl_sleep)
