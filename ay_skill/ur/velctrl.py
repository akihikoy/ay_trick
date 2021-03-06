#!/usr/bin/python
from core_tool import *
def Help():
  return '''Velocity control tools for UR.  Do not use this directly.
    Usage:
      velctrl= ct.Load('ur.velctrl').TVelCtrl(arm,ct)
      try:
        while ...:
          dq= [...]
          velctrl.Step(dq)
      finally:
        velctrl.Finish()
    '''
def Run(ct,*args):
  print 'Error:',Help()


'''
A low-level velocity control implementation of UR
with a direct connection to UR over Ethernet (no ROS topic is used).
FIXME:TODO: This is an experimental code.
The implementation of TUR* are general-purpose classes.
--> Move these codes to ay_py.misc.
'''

import socket,struct

class TURVersion(object):
  def __init__(self):
    self.timestamp= 0  #uint64_t
    self.source= 0  #int8_t
    self.robot_message_type= 0  #int8_t
    self.project_name_size= 0  #int8_t
    self.project_name= ''  #char[15]
    self.major_version= 0  #int8_t
    self.minor_version= 0  #int8_t
    self.svn_revision= 0  #int
    self.build_date= ''  #char[25]

class TURState(object):
  ROBOT_STATE= 16
  ROBOT_MESSAGE= 20
  PROGRAM_STATE_MESSAGE= 25
  ROBOT_MESSAGE_TEXT= 0
  ROBOT_MESSAGE_PROGRAM_LABEL= 1
  PROGRAM_STATE_MESSAGE_VARIABLE_UPDATE= 2
  ROBOT_MESSAGE_VERSION= 3
  ROBOT_MESSAGE_SAFETY_MODE= 5
  ROBOT_MESSAGE_ERROR_CODE= 6
  ROBOT_MESSAGE_KEY= 7
  ROBOT_MESSAGE_REQUEST_VALUE= 9
  ROBOT_MESSAGE_RUNTIME_EXCEPTION= 10

  def __init__(self):
    self.version= TURVersion()

  def GetVersion(self):
    return self.version.major_version + 0.1*self.version.minor_version + 0.0000001*self.version.svn_revision

  def Unpack(self, buf):
    offset= 0
    while len(buf)>offset:
      l,message_type= struct.unpack_from("!IB", buf, offset)
      if l+offset>len(buf):  return
      if message_type==self.ROBOT_MESSAGE:
        self.UnpackRobotMessage(buf, offset, l)
      elif message_type==self.ROBOT_STATE:
        #self.UnpackRobotState(buf, offset, l)
        pass
      if message_type==self.PROGRAM_STATE_MESSAGE:
        pass
      offset+= l

  def UnpackRobotMessage(self, buf, offset, l):
    offset+= 5
    timestamp,source,robot_message_type= struct.unpack_from("!Qbb", buf, offset)
    offset+= 8+1+1
    if robot_message_type==self.ROBOT_MESSAGE_VERSION:
      self.version.timestamp= timestamp
      self.version.source= source
      self.version.robot_message_type= robot_message_type
      self.version.project_name_size= struct.unpack_from("!b", buf, offset)[0]
      offset+= 1
      self.version.project_name= buf[offset:offset+self.version.project_name_size]
      offset+= self.version.project_name_size
      mj,mi,svn= struct.unpack_from("!bbI", buf, offset)
      self.version.major_version,self.version.minor_version,self.version.svn_revision= mj,mi,svn
      offset+= 1+1+4
      self.version.build_date= buf[offset:l]
      #if version_msg_.major_version<2:
        #robot_mode_running_= ROBOT_RUNNING_MODE

def GetURState(robot_hostname):
  port= 30001
  socketobj= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  socketobj.connect((robot_hostname, port))

  robot_state= TURState()
  buf= socketobj.recv(512)
  socketobj.close()
  robot_state.Unpack(buf)
  return robot_state

#Velocity control interface of UR.
class TURVelCtrl(object):
  def __init__(self, robot_hostname):
    self.robot_hostname= robot_hostname

  def Init(self):
    self.robot_state= GetURState(self.robot_hostname)
    self.robot_ver= self.robot_state.GetVersion()
    port= 30003
    #socketobj= socket.create_connection((robot_hostname, port))
    self.socketobj= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.socketobj.connect((self.robot_hostname, port))
    self.dt= None  #Control time step.
    #if self.robot_ver>=3.3:
      ##self.dt= 0.008
      ##self.dt= 0.016
      #self.dt= 0.02
      #'''NOTE: Since the robot is operated at 125 Hz, the control time step should be
      #0.008 sec.  However when the computer looses the real-time, the robot stops,
      #which causes a shaky motion.
      #To avoid this, we use a larger control time step.'''
    #elif self.robot_ver>=3.1:
      #pass
    #else:
      #self.dt= 0.02
      ##This value is from ur_modern_driver/src/ur_realtime_communication.cpp UrRealtimeCommunication::setSpeed
    #self.dt= 0.02  #NOTE@20190529: This value (0.02) was copied from the latest version of ur_modern_driver.
    self.dt= 0.04  #NOTE@20190531: Increased dt since there were frame drops in the control loop.
    self.Step([0.0]*6, 10.0)

  #Control step.  NOTE: This is expected to be running at 125 Hz.
  def Step(self, dq, acc):
    #This swich for the version was implemented previously as the handler of the joint_speed topic,
    #but as of 20190529 that topic is removed and the alternative hardware interface uses the following.
    #if self.robot_ver>=3.3:
      #cmd= "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, %f)\n" % (
              #dq[0],dq[1],dq[2],dq[3],dq[4],dq[5],acc,self.dt)
    #elif self.robot_ver>=3.1:
      #cmd= "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f)\n" % (
              #dq[0],dq[1],dq[2],dq[3],dq[4],dq[5],acc)
    #else:
      #cmd= "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, %f)\n" % (
              #dq[0],dq[1],dq[2],dq[3],dq[4],dq[5],acc,self.dt)
    cmd= "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, %f)\n" % (
            dq[0],dq[1],dq[2],dq[3],dq[4],dq[5],acc,self.dt)
    self.AddCommandToQueue(cmd)
    #CPrint(2,cmd)

  def Finish(self):
    if self.socketobj is None:  return
    self.Step([0.0]*6, 10.0)
    self.socketobj.close()
    self.socketobj= None

  def AddCommandToQueue(self, cmd):
    if cmd[-1]!='\n': cmd+= '\n'
    len_sent= self.socketobj.send(cmd)
    if len_sent!=len(cmd):
      raise Exception('Command was not sent properly; cmd {cmd} byte, sent {sent} byte'.format(cmd=len(cmd),sent=len_sent))

class TVelCtrl(object):
  __metaclass__= TMultiSingleton

  #ct: core_tool.
  #rate: Control time cycle in Hz.
  #dq_lim: Limit of joint angular velocity (rad/s).
  #ddq_lim: Limit of joint angular acceleration (rad/s**2).
  #q_limit_th: Threshold to detect if a joint angle is on the limit.
  def __init__(self, arm, ct, rate=None, dq_lim=40.0, ddq_lim=3.0, q_limit_th=0.02):
    #ddq_lim can be: lambda dq:3.0 if max(map(abs,dq))>0.1 else 1.0
    if rate is None:  rate= 500 if ct.robot.Is('E') else 125
    self.rate= rate
    self.dq_lim= dq_lim
    self.ddq_lim= ddq_lim
    self.q_limit_th= q_limit_th
    self.ct= ct
    self.arm= arm
    self.velctrl= TURVelCtrl(ct.robot.RobotIP())
    self.velctrl.Init()
    self.rate_adjuster= rospy.Rate(self.rate)
    self.last_dq= [0.0]*ct.robot.DoF(self.arm)
    #self.dbglogger= open('/tmp/urvctrl.log','w')
    #self.dbglogger_start= rospy.Time.now()

  def Rate(self):
    return self.rate
  def TimeStep(self):
    return 1.0/float(self.rate)

  #dq: Joint angular velocity (target; rad/s).
  #sleep: If True, this function waits until the end of the control cycle.
  def Step(self, dq, sleep=True):
    ct= self.ct
    arm= self.arm
    dof= ct.robot.DoF(arm)
    #dq= copy.deepcopy(dq)
    dq= [v if abs(v)<vmax else v/vmax for v,vmax in zip(dq,ct.robot.JointVelLimits(arm))]
    dq_original= copy.deepcopy(dq)
    dt= self.TimeStep()
    #print ' '.join(map(lambda f:'%0.2f'%f,dq))

    if not ct.robot.IsNormal():
      #self.Finish()  #Do not call self.Finish since it deletes the singleton instance.
      self.velctrl.Finish()
      raise Exception('TVelCtrl has stopped as the robot is not normal state.')

    if self.rate_adjuster.remaining().to_sec()<-self.velctrl.dt:
      CPrint(4,'Loosing real-time control(0):', self.rate_adjuster.remaining().to_sec())
      #self.last_dq= ct.robot.DQ(arm=arm)
      self.last_dq= [0.0]*dof

    #Get current state:
    q= ct.robot.Q(arm=arm)
    #dq0= ct.robot.DQ(arm=arm)
    dq0= self.last_dq
    '''NOTE
    We use last-dq (target velocities) rather than current actual velocities
    in order to avoid oscillation.
    '''
    #dq0= 0.9*np.array(self.last_dq)+0.1*np.array(ct.robot.DQ(arm=arm))

    #Limit accelerations:
    ddq_max= max([abs(v-v0)/dt for v,v0 in zip(dq,dq0)])
    ddq_lim= self.ddq_lim if not callable(self.ddq_lim) else self.ddq_lim(dq0)
    if ddq_max>ddq_lim:
      scale= ddq_lim/ddq_max
      #print dq, dq0, [(v-v0)/dt for v,v0 in zip(dq,dq0)], scale
      dq= [scale*v+(1.0-scale)*v0 for v,v0 in zip(dq,dq0)]
      CPrint(3,'ur.velctrl: dq is modified due to exceeding ddq_lim;',ddq_max,ddq_lim)
      #print scale,dq0,dq
    #Limit velocities:
    dq_max= max(map(abs,dq))
    if dq_max>self.dq_lim:
      scale= self.dq_lim/dq_max
      dq= [v*scale for v in dq]
      CPrint(3,'ur.velctrl: dq is modified due to exceeding dq_lim;',dq_max,self.dq_lim)

    q2= [0.0]*dof
    for j,(qi,dqi,qmini,qmaxi) in enumerate(zip(q,dq,*ct.robot.JointLimits(arm))):
      q2[j]= qi+dt*dqi
      if q2[j]<qmini+self.q_limit_th:
        q2[j]= qmini+self.q_limit_th
        dq[j]= (q2[j]-qi)/dt
        CPrint(3,'ur.velctrl: Joint angle is exceeding the limit;',qi,dqi,qmini,qmaxi)
      if q2[j]>qmaxi-self.q_limit_th:
        q2[j]= qmaxi-self.q_limit_th
        dq[j]= (q2[j]-qi)/dt
        CPrint(3,'ur.velctrl: Joint angle is exceeding the limit;',qi,dqi,qmini,qmaxi)

    #if self.rate_adjuster.remaining().to_sec()<0:
      #CPrint(4,'Loosing real-time control(1):', self.rate_adjuster.remaining().to_sec())
    self.velctrl.Step(dq, 100.0)
    self.last_dq= dq
    #if self.dbglogger is not None:
      #self.dbglogger.write('{time} {rate_remain} {dq} {dq_original} {robot_dq}\n'.format(time=(rospy.Time.now()-self.dbglogger_start).to_sec(),rate_remain=self.rate_adjuster.remaining().to_sec(),dq=ToStr(dq),dq_original=ToStr(dq_original),robot_dq=ToStr(ct.robot.DQ(arm=arm)) ))
    #print 'ur.velctrl:rate_adjuster.remaining:',self.rate_adjuster.remaining().to_sec()
    if self.rate_adjuster.remaining().to_sec()<-self.velctrl.dt:
      CPrint(4,'Loosing real-time control(2):', self.rate_adjuster.remaining().to_sec())
      #self.last_dq= ct.robot.DQ(arm=arm)
      self.last_dq= [0.0]*dof
    if sleep:  self.rate_adjuster.sleep()

  def Finish(self):
    self.__class__.Delete(self.arm)
    if self.__class__.NumReferences(self.arm)>0:  return
    self.velctrl.Finish()
    #self.dbglogger.close()
    #self.dbglogger= None

