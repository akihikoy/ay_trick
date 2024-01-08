#!/usr/bin/python
from core_tool import *
def Help():
  return '''Velocity control tools for kinematic simulation of Gen3.  Do not use this directly.
  This code uses the ROS topic /gen3a/joint_speed_command.
    Usage:
      velctrl= ct.Load('gen3.velctrl_sim').TVelCtrl(arm,ct)
      try:
        while ...:
          dq= [...]
          velctrl.Step(dq)
      finally:
        velctrl.Finish()
    '''
def Run(ct,*args):
  print 'Error:',Help()

class TVelCtrl(object):
  __metaclass__= TMultiSingleton

  #ct: core_tool.
  #rate: Control time cycle in Hz.
  #dq_lim: Limit of joint angular velocity (rad/s).
  #ddq_lim: Limit of joint angular acceleration (rad/s**2).
  #q_limit_th: Threshold to detect if a joint angle is on the limit.
  def __init__(self, arm, ct, rate=None, dq_lim=40.0, ddq_lim=3.0, q_limit_th=0.02):
    self.rate= rate if rate is not None else 125
    self.dq_lim= dq_lim
    self.ddq_lim= ddq_lim
    self.q_limit_th= q_limit_th
    self.ct= ct
    self.arm= arm
    ct.AddPub('trg_vel', '/gen3a/joint_speed_command', trajectory_msgs.msg.JointTrajectory, queue_size=2)
    self.rate_adjuster= rospy.Rate(self.rate)
    self.traj= trajectory_msgs.msg.JointTrajectory()
    self.traj.joint_names= ct.robot.JointNames(arm)
    self.traj.points= [trajectory_msgs.msg.JointTrajectoryPoint()]
    self.last_dq= [0.0]*ct.robot.DoF(self.arm)

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
    dq= copy.deepcopy(dq)
    dt= self.TimeStep()
    #print ' '.join(map(lambda f:'%0.2f'%f,dq))

    #Get current state:
    q= ct.robot.Q(arm=arm)
    #dq0= ct.robot.DQ(arm=arm)
    dq0= self.last_dq
    '''NOTE
    We use last dq (target velocities) rather than current actual velocities
    in order to avoid oscillation.
    '''

    #Limit accelerations:
    ddq_max= max([abs(v-v0)/dt for v,v0 in zip(dq,dq0)])
    if ddq_max>self.ddq_lim:
      scale= self.ddq_lim/ddq_max
      #print dq, dq0, [(v-v0)/dt for v,v0 in zip(dq,dq0)], scale
      dq= [scale*v+(1.0-scale)*v0 for v,v0 in zip(dq,dq0)]
      CPrint(3,'gen3.velctrl_sim: dq is modified due to exceeding ddq_lim;',ddq_max,self.ddq_lim)
      #print scale,dq0,dqtmp,dq
    #Limit velocities:
    dq_max= max(map(abs,dq))
    if dq_max>self.dq_lim:
      scale= self.dq_lim/dq_max
      dq= [v*scale for v in dq]
      CPrint(3,'gen3.velctrl_sim: dq is modified due to exceeding dq_lim;',dq_max,self.dq_lim)

    q2= [0.0]*dof
    for j,(qi,dqi,qmini,qmaxi) in enumerate(zip(q,dq,*ct.robot.JointLimits(arm))):
      q2[j]= qi+dt*dqi
      if q2[j]<qmini+self.q_limit_th:
        q2[j]= qmini+self.q_limit_th
        dq[j]= (q2[j]-qi)/dt
        CPrint(3,'gen3.velctrl_sim: Joint angle is exceeding the limit;',qi,dqi,qmini,qmaxi)
      if q2[j]>qmaxi-self.q_limit_th:
        q2[j]= qmaxi-self.q_limit_th
        dq[j]= (q2[j]-qi)/dt
        CPrint(3,'gen3.velctrl_sim: Joint angle is exceeding the limit;',qi,dqi,qmini,qmaxi)

    self.traj.points[0].velocities= dq
    self.traj.points[0].accelerations= [100.0]*dof

    ct.pub.trg_vel.publish(self.traj)
    self.last_dq= dq
    #print self.rate_adjuster.remaining().to_sec()
    if self.rate_adjuster.remaining().to_sec()<0:
      CPrint(4,'Loosing real-time control:', self.rate_adjuster.remaining().to_sec())
    if sleep:  self.rate_adjuster.sleep()

  def Finish(self):
    self.__class__.Delete(self.arm)
    if self.__class__.NumReferences(self.arm)>0:  return
    ct= self.ct
    self.traj.points[0].velocities= [0.0]*ct.robot.DoF(self.arm)
    ct.pub.trg_vel.publish(self.traj)
