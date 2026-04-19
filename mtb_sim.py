# Import Libraries
import numpy as np
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

# Connect to CoppeliaSim
print('Connecting...')
client = RemoteAPIClient('localhost', 23000)
sim = client.require('sim')
print('Connected')
sim.setBoolParam(sim.boolparam_realtime_simulation, True)

# Load robot
robot = sim.loadModel(r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu\models\robots\non-mobile\MTB robot.ttm')
print('Robot loaded')

# Remove default script
script = sim.getScript(sim.scripttype_childscript, robot)
suction_script = sim.getScript(sim.scripttype_childscript, sim.getObject('/MTB/suctionPad'))
default_box = sim.getObject('/MTB/Rectangle')

if script != -1 and suction_script != -1:
    sim.removeObjects([script, suction_script, default_box])

# Assign MTB joints
q1 = sim.getObject('/MTB/axis')
q2 = sim.getObject('/MTB/axis/link/axis')
q3 = sim.getObject('/MTB/axis/link/axis/link/axis')
q4 = sim.getObject('/MTB/axis/link/axis/link/axis/axis')
tool = sim.getObject('/MTB/suctionPad/Body')
print('Joints assigned')

# Get positions of the joints
o1 = np.array(sim.getObjectPosition(q1, sim.handle_world))
o2 = np.array(sim.getObjectPosition(q2, sim.handle_world))
o3 = np.array(sim.getObjectPosition(q3, sim.handle_world))

# Compute link lengths
l1 = np.linalg.norm(o2-o1)
l2 = np.linalg.norm(o3-o2)
print(f'l1={l1}, l2={l2}, o1={o1}, o2={o2}, o3={o3}')

# Define a Box class that will be used to create boxes in the simulation
class Box:
    def __init__(self, sim, position, size=[0.1, 0.1, 0.1], velocity=[0.1, 0, 0]):
        self.sim = sim
        self.size = size
        self.velocity = np.array(velocity)
        self.attached = False
        self.pos = position

        self.handle = sim.createPrimitiveShape(
            sim.primitiveshape_cuboid,
            size,
            0
        )

        sim.setObjectPosition(self.handle, list(self.pos), sim.handle_world)
        sim.setObjectInt32Param(self.handle, sim.shapeintparam_static, 1)
        sim.setShapeMass(self.handle, 0.1)
        sim.setShapeColor(self.handle, None, sim.colorcomponent_ambient_diffuse, [0.65, 0.65, 1])
    
    def update(self, dt):
        if not self.attached:
            pos = np.array(self.sim.getObjectPosition(self.handle, self.sim.handle_world))
            new_pos = pos + self.velocity * dt
            self.sim.setObjectPosition(self.handle, list(new_pos), self.sim.handle_world)
    
    def attach(self, tool_handle):
        self.sim.setObjectParent(self.handle, tool_handle, True)
        self.attached = True

    def detach(self):
        self.sim.setObjectParent(self.handle, -1, True)
        self.attached = False

# Inverse kinematics is handled by CoppeliaSim's IK solver

# Start the simulation
print('Starting simulation...')
sim.setStepping(True)
sim.startSimulation()
dt = sim.getSimulationTimeStep()

#~~~~~~~~~~~~~~~~~~~~~~~~~
# TODO SIMULATION

boxes = []
boxes.append(Box(sim, position=[0.5, .5, 0.05]))

box = boxes[0]

sim.setJointTargetPosition(q1, np.radians(-30))
sim.setJointTargetPosition(q2, np.radians(-20))
sim.setJointTargetPosition(q3, 0.1)

for i in range(200):
    # box.update(dt)
    if i == 50:
        # Get current box position
        box_pos = sim.getObjectPosition(box.handle, sim.handle_world)
        x, y, z = box_pos
        print(f'Box position at step {i}: {box_pos}')
        
        # Compute relative to base
        x_rel = x - o1[0]
        y_rel = y - o1[1]
        z_rel = z - o1[2]
        print(f'Relative position: {x_rel}, {y_rel}, {z_rel}')
        
        # Compute inverse kinematics to reach the box
        q1_target, q2_target, q3_target = inverse_kinematics(x_rel, y_rel, z_rel, l1, l2)
        print(f'Target joints: q1={np.degrees(q1_target)}, q2={np.degrees(q2_target)}, q3={q3_target}')
        sim.setObjectFloatParam(q1, sim.jointfloatparam_maxvel, 10)
        sim.setObjectFloatParam(q2, sim.jointfloatparam_maxvel, 10)
        sim.setObjectFloatParam(q3, sim.jointfloatparam_maxvel, 10)
        sim.setJointTargetPosition(q1, q1_target)
        sim.setJointTargetPosition(q2, q2_target)
        sim.setJointTargetPosition(q3, q3_target)
        
        tool_pos = sim.getObjectPosition(tool, sim.handle_world)
        print(f'The tool position is at {tool_pos}')
        
        box.attach(tool)

    # if i == 100:
    #     sim.setJointTargetPosition(q1, np.radians(30))
    #     sim.setJointTargetPosition(q2, np.radians(20))

    # if i == 120:
    #     box.detach()

    sim.step()
    
    if i == 50:
        tool_pos = sim.getObjectPosition(tool, sim.handle_world)
        print(f'Tool position after move at step {i}: {tool_pos}')

# u1 = np.radians(-30)
# u2 = np.radians(-20)
# u3 = 0.0

# sim.setJointTargetPosition(q1, u1)
# sim.setJointTargetPosition(q2, u2)
# sim.setJointTargetPosition(q3, u3)

# for _ in range(60):
#     sim.step()

# u1 = np.radians(30)
# u2 = np.radians(-20)
# u3 = 0.10

# sim.setJointTargetPosition(q1, u1)
# sim.setJointTargetPosition(q2, u2)
# sim.setJointTargetPosition(q3, u3)

# for _ in range(60):
#     sim.step()
#~~~~~~~~~~~~~~~~~~~~~~~~~

# Stop the simulation
sim.stopSimulation()
sim.setStepping(False)
print('Stopped')

# Remove MTB model
sim.removeModel(robot)
