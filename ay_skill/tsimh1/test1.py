#!/usr/bin/python3
from core_tool import *
def Help():
  return '''Test of ODE hopping robot simulation.
  Usage: tsimh1.test1'''

def Run(ct,*args):
  l= TContainer(debug=True)

  sim= ct.Load('tsimh1.core1')

  try:
    sim.SetupServiceProxy(ct,l)
    sim.SetupPubSub(ct,l)

    ct.srvp.ode_resume()
    l.config= sim.GetConfig(ct)
    print('Current config:',l.config)

    #Setup config
    #l.config.BaseDensity= 500.0

    #Reset with sending config
    sim.ResetConfig(ct,l.config)
    time.sleep(0.1)  #Wait for l.sensors is updated

    print('l.sensors:',l.sensors)

    #Jump motion (with slightly twisting):
    for i in range(30):
      #sim.MoveDTheta(ct,l,[0.0,0.0,0.4,-0.7])
      sim.MoveDTheta(ct,l,[0.2,0.2,0.4,-0.7])
      print(l.sensors.Time,l.sensors.JointAngles)
    for i in range(15):
      #sim.MoveDTheta(ct,l,[0.0,0.0,-0.8,1.4])
      sim.MoveDTheta(ct,l,[-0.4,-0.4,-0.8,1.4])
      print(l.sensors.Time,l.sensors.JointAngles)

    time.sleep(2.0)  #Observe

    #Reset with sending config
    sim.ResetConfig(ct,l.config)
    time.sleep(0.1)  #Wait for l.sensors is updated

    #Jump motion (2):
    tq_traj= [[0.0, [0.0,0.0,0.0,0.0]],
              [0.5, [0.0,0.0,1.8,-1.9]],
              [0.7, [0.0,0.0,0.0,0.0]]]
    sim.TrackTraj(ct,l,q_traj=[q for tm,q in tq_traj],t_traj=[tm for tm,q in tq_traj])

    time.sleep(2.0)  #Observe

  except Exception as e:
    PrintException(e, ' in tsim.test1')

  finally:
    sim.StopPubSub(ct,l)
    ct.srvp.ode_pause()
