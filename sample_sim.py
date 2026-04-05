'''
Sample Python script to run a simulation in CoppeliaSim
To be used as a starting point for the project
'''

import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Connect to CoppeliaSim
print('Connecting...')
client = RemoteAPIClient('localhost', 23000)
sim = client.require('sim')
print('Connected')

# Start the simulation
print('Starting simulation...')
sim.startSimulation()

time.sleep(2)  # Placeholder

sim_time = sim.getSimulationTime()
print('Simulation time:', sim_time)

# Stop the simulation
sim.stopSimulation()
print('Stopped')
