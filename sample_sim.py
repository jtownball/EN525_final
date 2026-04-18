'''
Sample Python script to run a simulation in CoppeliaSim
To be used as a starting point for the project
'''

import numpy as np
import math
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Connect to CoppeliaSim
print('Connecting...')
client = RemoteAPIClient('localhost', 23000)
sim = client.require('sim')
print('Connected')

# Assign MTB joints
q1 = sim.getObject('/MTB/axis')
q2 = sim.getObject('/MTB/axis/link/axis')
q3 = sim.getObject('/MTB/axis/link/axis/link/axis')
q4 = sim.getObject('/MTB/axis/link/axis/link/axis/axis')
print('Joints assigned')


# Start the simulation
print('Starting simulation...')
sim.setStepping(True)
sim.startSimulation()

try:
    u1 = math.radians(30)
    u2 = math.radians(-20)
    u3 = 0.10

    sim.setJointTargetPosition(q1, u1)
    sim.setJointTargetPosition(q2, u2)
    sim.setJointTargetPosition(q3, u3)

    for _ in range(100):
        sim.step()

finally:
    sim_time = sim.getSimulationTime()
    print('Simulation time:', sim_time)

    # Stop the simulation
    sim.stopSimulation()
    print('Stopped')
