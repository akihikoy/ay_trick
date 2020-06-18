#!/usr/bin/python
from core_tool import *
import tf
def Help():
  return '''Robot initialization utility.
  Usage: robot ROBOT_NAME
    Available ROBOT_NAME:
      'NoRobot',
      'pr2','PR2',
      'pr2s','PR2_SIM',
      'bx','Baxter',
      'bxs','Baxter_SIM',
      'bxn','BaxterN',
      'rq','RobotiqNB',
      'dxlg','DxlGripper',
      'thg','RHP12RNGripper',
      'ezg','EZGripper',
      'dxlpo2_st1','DxlpO2Gripper_Straight1',
      'dxlpo2_sr1','DxlpO2Gripper_SRound1',
      'dxlpo2_f1' ,'DxlpO2Gripper_Fork1',
      'dxlo3','DxlO3Gripper',
      'moto','Motoman',
      'motos','Motoman_SIM',
      'mikata','Mikata',
      'mikatas','Mikata_SIM',
      'mikata2','Mikata2',
      'mikata6','Mikata6',
      'mikata6s','Mikata6_SIM',
      'cx7','CraneX7',
      'ur3','UR3',
      'ur3s','UR3_SIM',
      'ur3dxlg','UR3DxlG',
      'ur3dxlgs','UR3DxlG_SIM',
      'ur3thg','UR3ThG',
      'ur3thgs','UR3ThG_SIM',
      'ur3dxlpo2_st1','UR3DxlpO2_Straight1',
      'ur3dxlpo2_sr1','UR3DxlpO2_SRound1',
      'ur3dxlpo2_f1' ,'UR3DxlpO2_Fork1',
      'ur3dxlpo2_st1s','UR3DxlpO2_Straight1_SIM',
      'ur3dxlpo2_sr1s','UR3DxlpO2_SRound1_SIM',
      'ur3dxlpo2_f1s' ,'UR3DxlpO2_Fork1_SIM',
      'ur3e','UR3e',
      'ur3es','UR3e_SIM',
      'ur3ethg','UR3eThG',
      'ur3ethgs','UR3eThG_SIM',
      'ur3edxlg','UR3eDxlG',
      'ur3edxlgs','UR3eDxlG_SIM',
      'ur3edxlpo2_st1','UR3eDxlpO2_Straight1',
      'ur3edxlpo2_sr1','UR3eDxlpO2_SRound1',
      'ur3edxlpo2_f1' ,'UR3eDxlpO2_Fork1',
      'ur3edxlpo2_st1s','UR3eDxlpO2_Straight1_SIM',
      'ur3edxlpo2_sr1s','UR3eDxlpO2_SRound1_SIM',
      'ur3edxlpo2_f1s' ,'UR3eDxlpO2_Fork1_SIM',
      'ur5e','UR5e',
      'ur5es','UR5e_SIM',
      'ur5ethg','UR5eThG',
      'ur5ethgs','UR5eThG_SIM',
      'gen3','Gen3',
      'gen3s','Gen3_SIM',
      'gen3thg','Gen3ThG',
      'gen3thgs','Gen3ThG_SIM',
      'gen3dxlo3','Gen3DxlO3',
      'gen3dxlo3s','Gen3DxlO3_SIM',
    If ROBOT_NAME is omitted, we assume 'NoRobot'.

  '''
def Run(ct,*args):
  robot= args[0] if len(args)>0 else 'NoRobot'

  alias={
      'pr2':'PR2',
      'pr2s':'PR2_SIM',
      'bx':'Baxter',
      'bxs':'Baxter_SIM',
      'bxn':'BaxterN',
      'rq':'RobotiqNB',
      'dxlg':'DxlGripper',
      'thg':'RHP12RNGripper',
      'ezg':'EZGripper',
      'dxlpo2_st1':'DxlpO2Gripper_Straight1',
      'dxlpo2_sr1':'DxlpO2Gripper_SRound1',
      'dxlpo2_f1' :'DxlpO2Gripper_Fork1',
      'dxlo3':'DxlO3Gripper',
      'moto':'Motoman',
      'motos':'Motoman_SIM',
      'mikata':'Mikata',
      'mikatas':'Mikata_SIM',
      'mikata2':'Mikata2',
      'mikata6':'Mikata6',
      'mikata6s':'Mikata6_SIM',
      'cx7':'CraneX7',
      'ur3':'UR3',
      'ur3s':'UR3_SIM',
      'ur3dxlg':'UR3DxlG',
      'ur3dxlgs':'UR3DxlG_SIM',
      'ur3thg':'UR3ThG',
      'ur3thgs':'UR3ThG_SIM',
      'ur3dxlpo2_st1':'UR3DxlpO2_Straight1',
      'ur3dxlpo2_sr1':'UR3DxlpO2_SRound1',
      'ur3dxlpo2_f1' :'UR3DxlpO2_Fork1',
      'ur3dxlpo2_st1s':'UR3DxlpO2_Straight1_SIM',
      'ur3dxlpo2_sr1s':'UR3DxlpO2_SRound1_SIM',
      'ur3dxlpo2_f1s' :'UR3DxlpO2_Fork1_SIM',
      'ur3e':'UR3e',
      'ur3es':'UR3e_SIM',
      'ur3ethg':'UR3eThG',
      'ur3ethgs':'UR3eThG_SIM',
      'ur3edxlg':'UR3eDxlG',
      'ur3edxlgs':'UR3eDxlG_SIM',
      'ur3edxlpo2_st1':'UR3eDxlpO2_Straight1',
      'ur3edxlpo2_sr1':'UR3eDxlpO2_SRound1',
      'ur3edxlpo2_f1' :'UR3eDxlpO2_Fork1',
      'ur3edxlpo2_st1s':'UR3eDxlpO2_Straight1_SIM',
      'ur3edxlpo2_sr1s':'UR3eDxlpO2_SRound1_SIM',
      'ur3edxlpo2_f1s' :'UR3eDxlpO2_Fork1_SIM',
      'ur5e':'UR5e',
      'ur5es':'UR5e_SIM',
      'ur5ethg':'UR5eThG',
      'ur5ethgs':'UR5eThG_SIM',
      'gen3':'Gen3',
      'gen3s':'Gen3_SIM',
      'gen3thg':'Gen3ThG',
      'gen3thgs':'Gen3ThG_SIM',
      'gen3dxlo3':'Gen3DxlO3',
      'gen3dxlo3s':'Gen3DxlO3_SIM',
    }
  if robot in alias:  robot= alias[robot]

  print 'Setup robot for',robot

  if ct.state_validity_checker is not None:
    del ct.state_validity_checker
    ct.state_validity_checker= None
  if ct.robot is not None and not ct.robot.Is('NoRobot'):
    ct.Run('fv.fv','clear')

    ct.robot.Cleanup()
    del ct.robot
    ct.robot= None

  if   robot in ('PR2','PR2_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_pr2')
    ct.robot= mod.TRobotPR2()

  elif robot in ('Baxter','Baxter_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_bxtr')
    ct.robot= mod.TRobotBaxter(is_sim=(robot=='Baxter_SIM'))

  elif robot in ('BaxterN',):
    mod= SmartImportReload('ay_py.ros.rbt_bxtrN')
    ct.robot= mod.TRobotBaxterN()

  elif robot in ('RobotiqNB',):
    mod= SmartImportReload('ay_py.ros.rbt_rqnb')
    ct.robot= mod.TRobotRobotiqNB()

  elif robot in ('DxlGripper',):
    mod= SmartImportReload('ay_py.ros.rbt_dxlg')
    ct.robot= mod.TRobotDxlGripper()

  elif robot in ('RHP12RNGripper',):
    mod= SmartImportReload('ay_py.ros.rbt_rhp12rn')
    ct.robot= mod.TRobotRHP12RNGripper()

  elif robot in ('EZGripper',):
    mod= SmartImportReload('ay_py.ros.rbt_ezg')
    ct.robot= mod.TRobotEZGripper()

  elif robot in ('DxlpO2Gripper_Straight1','DxlpO2Gripper_SRound1','DxlpO2Gripper_Fork1'):
    mod= SmartImportReload('ay_py.ros.rbt_dxlpo2')
    ct.robot= mod.TRobotDxlpO2Gripper(finger_type=robot.split('_')[1])

  elif robot in ('DxlO3Gripper',):
    mod= SmartImportReload('ay_py.ros.rbt_dxlo3')
    ct.robot= mod.TRobotDxlO3Gripper()

  elif robot in ('Motoman','Motoman_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_moto')
    ct.robot= mod.TRobotMotoman(is_sim=(robot=='Motoman_SIM'))

  elif robot in ('Mikata2',):
    mod= SmartImportReload('ay_py.ros.rbt_mikata2')
    ct.robot= mod.TRobotMikata2()

  elif robot in ('Mikata6','Mikata6_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_mikata6')
    ct.robot= mod.TRobotMikata6(is_sim=(robot=='Mikata6_SIM'))

  elif robot in ('CraneX7',):
    mod= SmartImportReload('ay_py.ros.rbt_cranex7')
    ct.robot= mod.TRobotCraneX7()

  elif robot in ('UR3','UR3_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_ur')
    ct.robot= mod.TRobotUR(name='UR3',ur_series='CB',is_sim=(robot=='UR3_SIM'))

  elif robot in ('UR3DxlG','UR3DxlG_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_urdxlg')
    ct.robot= mod.TRobotURDxlG(name='UR3DxlG',ur_series='CB',is_sim=(robot=='UR3DxlG_SIM'))

  elif robot in ('UR3ThG','UR3ThG_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_urthg')
    ct.robot= mod.TRobotURThG(name='UR3ThG',ur_series='CB',is_sim=(robot=='UR3ThG_SIM'))

  elif robot in ('UR3DxlpO2_Straight1','UR3DxlpO2_SRound1','UR3DxlpO2_Fork1', 'UR3DxlpO2_Straight1_SIM','UR3DxlpO2_SRound1_SIM','UR3DxlpO2_Fork1_SIM'):
    codes= robot.split('_')
    robot_code= codes[0]
    finger_type= codes[1]
    is_sim= (len(codes)>=3 and codes[2]=='SIM')
    mod= SmartImportReload('ay_py.ros.rbt_urdxlpo2')
    ct.robot= mod.TRobotURDxlpO2(name='UR3DxlpO2',ur_series='CB',finger_type=finger_type,is_sim=is_sim)

  elif robot in ('UR3e','UR3e_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_ur')
    ct.robot= mod.TRobotUR(name='UR3e',ur_series='E',is_sim=(robot=='UR3e_SIM'))

  elif robot in ('UR3eThG','UR3eThG_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_urthg')
    ct.robot= mod.TRobotURThG(name='UR3eThG',ur_series='E',is_sim=(robot=='UR3eThG_SIM'))

  elif robot in ('UR3eDxlG','UR3eDxlG_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_urdxlg')
    ct.robot= mod.TRobotURDxlG(name='UR3eDxlG',ur_series='E',is_sim=(robot=='UR3eDxlG_SIM'))

  elif robot in ('UR3eDxlpO2_Straight1','UR3eDxlpO2_SRound1','UR3eDxlpO2_Fork1', 'UR3eDxlpO2_Straight1_SIM','UR3eDxlpO2_SRound1_SIM','UR3eDxlpO2_Fork1_SIM'):
    codes= robot.split('_')
    robot_code= codes[0]
    finger_type= codes[1]
    is_sim= (len(codes)>=3 and codes[2]=='SIM')
    mod= SmartImportReload('ay_py.ros.rbt_urdxlpo2')
    ct.robot= mod.TRobotURDxlpO2(name='UR3eDxlpO2',ur_series='E',finger_type=finger_type,is_sim=is_sim)

  elif robot in ('UR5e','UR5e_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_ur')
    ct.robot= mod.TRobotUR(name='UR5e',ur_series='E',is_sim=(robot=='UR5e_SIM'))

  elif robot in ('UR5eThG','UR5eThG_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_urthg')
    ct.robot= mod.TRobotURThG(name='UR5eThG',ur_series='E',is_sim=(robot=='UR5eThG_SIM'))

  elif robot in ('Gen3','Gen3_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_gen3')
    ct.robot= mod.TRobotGen3(gen3ns='gen3a', is_sim=(robot=='Gen3_SIM'))

  elif robot in ('Gen3ThG','Gen3ThG_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_gen3thg')
    ct.robot= mod.TRobotGen3ThG(gen3ns='gen3a', is_sim=(robot=='Gen3ThG_SIM'))

  elif robot in ('Gen3DxlO3','Gen3DxlO3_SIM'):
    mod= SmartImportReload('ay_py.ros.rbt_gen3dxlo3')
    ct.robot= mod.TRobotGen3DxlO3(gen3ns='gen3a', is_sim=(robot=='Gen3DxlO3_SIM'))

  elif robot=='NoRobot':
    ct.robot= TFakeRobot()
  else:
    raise Exception('Unknown robot: %s'%robot)

  ct.br= tf.TransformBroadcaster()
  if robot=='NoRobot' or ct.robot is None:  return

  robots_with_state_validity_checker= ('PR2','Baxter','Motoman','Mikata','UR','Gen3')
  if any([ct.robot.Is(rbt) for rbt in robots_with_state_validity_checker]):
    ct.state_validity_checker= TStateValidityCheckerMI()
  else:
    ct.state_validity_checker= None

  res= []
  ra= lambda r: res.append(r)

  ra(ct.robot.Init())
  if ct.state_validity_checker is not None:
    ra(ct.state_validity_checker.Init(ct.robot))

  if False in res:
    CPrint(4, 'Failed to setup robot:',robot)

  # Is this attribute ('environment') used?? --> OBSOLETE_CANDIDATE
  if not ct.robot.Is('sim'):
    ct.SetAttr('environment', 'real')
  elif ct.robot.Is('sim'):
    ct.SetAttr('environment', 'sim')

  #ct.Run('model_loader')

  model_dir= os.path.dirname(__file__)+'/data'
  if ct.robot.Is('PR2'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_pr2.yaml'))
  elif ct.robot.Is('Baxter'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_bx.yaml'))
  elif ct.robot.Is('RobotiqNB'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_rqnb.yaml'))
  elif ct.robot.Is('DxlGripper'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_dxlg.yaml'))
  elif ct.robot.Is('RHP12RNGripper'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_thg.yaml'))
  elif ct.robot.Is('EZGripper'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ezg.yaml'))
  elif ct.robot.Is('DxlpO2Gripper') and robot=='DxlpO2Gripper_Straight1':
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_dxlpo2_st1.yaml'))
  elif ct.robot.Is('DxlpO2Gripper') and robot=='DxlpO2Gripper_SRound1':
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_dxlpo2_sr1.yaml'))
  elif ct.robot.Is('DxlpO2Gripper') and robot=='DxlpO2Gripper_Fork1':
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_dxlpo2_f1.yaml'))
  elif ct.robot.Is('DxlO3Gripper'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_dxlo3.yaml'))
  elif ct.robot.Is('Motoman'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_moto.yaml'))
  elif ct.robot.Is('Mikata'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_mikata.yaml'))
  elif ct.robot.Is('UR3DxlG'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ur3dxlg.yaml'))
  elif ct.robot.Is('UR3ThG'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ur3thg.yaml'))
  elif ct.robot.Is('UR3DxlpO2') and robot=='UR3DxlpO2_Straight1':
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ur3dxlpo2_st1.yaml'))
  elif ct.robot.Is('UR3DxlpO2') and robot=='UR3DxlpO2_SRound1':
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ur3dxlpo2_sr1.yaml'))
  elif ct.robot.Is('UR3DxlpO2') and robot=='UR3DxlpO2_Fork1':
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ur3dxlpo2_f1.yaml'))
  elif ct.robot.Is('UR3eThG'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ur3ethg.yaml'))
  elif ct.robot.Is('UR3eDxlG'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ur3edxlg.yaml'))
  elif ct.robot.Is('UR3eDxlpO2') and robot=='UR3eDxlpO2_Straight1':
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ur3edxlpo2_st1.yaml'))
  elif ct.robot.Is('UR3eDxlpO2') and robot=='UR3eDxlpO2_SRound1':
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ur3edxlpo2_sr1.yaml'))
  elif ct.robot.Is('UR3eDxlpO2') and robot=='UR3eDxlpO2_Fork1':
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ur3edxlpo2_f1.yaml'))
  elif ct.robot.Is('UR5eThG'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_ur5ethg.yaml'))
  elif ct.robot.Is('Gen3ThG'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_gen3thg.yaml'))
  elif ct.robot.Is('Gen3DxlO3'):
    ct.AddDictAttr(LoadYAML(model_dir+'/robot/gripper_gen3dxlo3.yaml'))
  ct.SetAttr('default_frame', ct.robot.BaseFrame)
