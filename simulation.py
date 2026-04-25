###        EN.535.630 Final Project        ###
### Jeremy Ball, Omer Bowman, Richard Ngai ###

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
q = [q1, q2, q3, q4]
print('Joints assigned')

# Get positions of the joints
o1 = np.array(sim.getObjectPosition(q1, sim.handle_world))  # Should be at about (0,0,z)
o2 = np.array(sim.getObjectPosition(q2, sim.handle_world))
o3 = np.array(sim.getObjectPosition(q3, sim.handle_world))

# Compute link lengths
l1 = o2[0]-o1[0]  # 0.467m
l2 = o3[0]-o2[0]  # 0.400m

# Get maximun suction pad actuation
tool = sim.getObject('/MTB/suctionPad/Link')
o4 = np.array(sim.getObjectPosition(tool, sim.handle_world))
q3_max = o4[2]
q3_up = 0.0

# Define Box class
class Box:
    def __init__(self, sim, position: list, size: list=[0.1, 0.1, 0.1], velocity: list=[0.1, 0, 0], max_render: float=2.0):
        self.sim = sim
        self.size = size
        self.velocity = np.array(velocity)
        self.attached = False
        self.max_render = max_render

        self.handle = sim.createPrimitiveShape(
            sim.primitiveshape_cuboid,
            size,
            0
        )

        sim.setObjectPosition(self.handle, sim.handle_world, list(position))
        sim.setObjectInt32Param(self.handle, sim.shapeintparam_static, 1)
        sim.setShapeMass(self.handle, 0.1)
        sim.setShapeColor(self.handle, None, sim.colorcomponent_ambient_diffuse, [0.65, 0.65, 1])
    
    def update(self, dt):
        if self.handle == -1:
            return
        
        if not self.attached:
            pos = np.array(self.sim.getObjectPosition(self.handle, self.sim.handle_world))
            new_pos = pos + self.velocity * dt
            self.sim.setObjectPosition(self.handle, self.sim.handle_world, list(new_pos))

            # Automatically delete the box when it gets too far from the robot
            dist = np.sqrt(new_pos[0]**2 + new_pos[1]**2)
            if dist > self.max_render:
                self.delete()
    
    def attach(self, tool_handle):
        if self.handle == -1:
            return

        self.sim.setObjectParent(self.handle, tool_handle, True)
        self.attached = True

    def detach(self):
        if self.handle == -1:
            return

        self.sim.setObjectParent(self.handle, -1, True)
        self.attached = False
    
    def delete(self):
        if self.handle == -1:
            return
        try:
            # Detach if still parented
            parent = self.sim.getObjectParent(self.handle)
            if parent != -1:
                self.sim.setObjectParent(self.handle, -1, True)

            self.sim.removeObjects([self.handle])
            self.handle = -1
        
        except Exception as e:
            print(f"Failed to delete box: {e}")

# Compute inverse kinematics
def inverse_kinematics(x: float, y: float, l1: float, l2: float) -> tuple:
    # Analytical SCARA IK using the paper's formulas.
    # θ2 = ± arccos([x^2 + y^2 - l1^2 - l2^2]/[2*l1*l2])
    # θ1 = atan2(y, x) - atan2(l2*sin(θ2), l1 + l2*cos(θ2))

    # Ensure target (x,y) is in robot's range
    r = np.sqrt(x**2 + y**2)
    if r < abs(l1 - l2) or r > (l1 + l2):
        raise ValueError(f'Target ({x:.3f}, {y:.3f}) is out of reach for l1={l1:.3f}, l2={l2:.3f}')

    cos_q2 = (x**2 + y**2 - l1**2 - l2**2) / (2*l1*l2)
    cos_q2 = np.clip(cos_q2, -1.0, 1.0)
    q2 = np.arccos(cos_q2)

    # Switch elbow configuration if on the "right" side of the robot
    if y < 0:
        q2 = -q2

    q1 = np.arctan2(y,x) - np.arctan2(l2*np.sin(q2), l1+l2*np.cos(q2))

    return q1, q2

# Assign joint actions
def move_arm(sim, q: list, u: list):
    for i in range(len(q)):
        sim.setJointTargetPosition(q[i], u[i])

# Start the simulation
print('Starting simulation...')
sim.setStepping(True)
sim.startSimulation()
dt = sim.getSimulationTimeStep()


#~~~~~~~~~~~~~~~~~~~~~~~~~
# SIMULATION

boxes = []
num_boxes = 10
handled_count = 0


def spawn_box():
    box_y_start = np.random.uniform(-0.6, -0.3)
    box_size = []
    for _ in range(3):
        box_size.append(np.random.uniform(0.06, 0.11))

    box = Box(
        sim,
        position=[-0.03, box_y_start, box_size[2]/2],
        size=box_size,
        velocity=[0.1, 0.0, 0.0],
        max_render=2.0
    )
    boxes.append(box)
    return box

# Move all boxes 
def update_all_boxes(exclude_box=None):
    for box in boxes[:]:
        if box is exclude_box:
            continue

        box.update(dt)

        if box.handle == -1:
            boxes.remove(box)

# Perform loop 'num_boxes' amount of times
while handled_count < num_boxes:
    active_box = spawn_box()
    box_vel = active_box.velocity[:2].copy()
    q3_down = q3_max - active_box.size[2]

    # Follow the box
    for _ in range(50):
        update_all_boxes()
        box_pos = np.array(sim.getObjectPosition(active_box.handle, sim.handle_world))
        u1, u2 = inverse_kinematics(x=box_pos[0], y=box_pos[1], l1=l1, l2=l2)
        u = [u1, u2, q3_up, -(u1+u2)]
        move_arm(sim, q=q, u=u)
        sim.step()

    # Follow the box and lower the suction pad
    for _ in range(30):
        update_all_boxes()
        box_pos = np.array(sim.getObjectPosition(active_box.handle, sim.handle_world))
        u1, u2 = inverse_kinematics(x=box_pos[0], y=box_pos[1], l1=l1, l2=l2)
        u = [u1, u2, q3_down, -(u1+u2)]
        move_arm(sim, q=q, u=u)
        sim.step()

    # Attach the box to the tool
    active_box.attach(tool)
    xy_pos = sim.getObjectPosition(active_box.handle, sim.handle_world)[:2]

    # Raise the box but keep forward motion
    for _ in range(30):
        update_all_boxes()
        xy_pos[0] = xy_pos[0] + box_vel[0]*dt
        u1, u2 = inverse_kinematics(x=xy_pos[0], y=xy_pos[1], l1=l1, l2=l2)
        u = [u1, u2, q3_up, -(u1+u2)]
        move_arm(sim, q=q, u=u)
        sim.step()

    # Drop-off point
    xy_pos = [-0.3, 0.65]

    # Move to the drop off point and match conveyor velocity
    for _ in range(60):
        update_all_boxes()
        xy_pos[0] = xy_pos[0] + box_vel[0]*dt
        u1, u2 = inverse_kinematics(x=xy_pos[0], y=xy_pos[1], l1=l1, l2=l2)
        u = [u1, u2, q3_up, -(u1+u2)]
        move_arm(sim, q=q, u=u)
        sim.step()

    # Lower box at conveyor velocity
    for _ in range(30):
        update_all_boxes()
        xy_pos[0] = xy_pos[0] + box_vel[0]*dt
        u1, u2 = inverse_kinematics(x=xy_pos[0], y=xy_pos[1], l1=l1, l2=l2)
        u = [u1, u2, q3_down, -(u1+u2)]
        move_arm(sim, q=q, u=u)
        sim.step()

    # Detach box
    active_box.detach()
    handled_count += 1

    # Raise and follow for a bit
    for _ in range(20):
        update_all_boxes()
        box_pos = np.array(sim.getObjectPosition(active_box.handle, sim.handle_world))
        u1, u2 = inverse_kinematics(x=box_pos[0], y=box_pos[1], l1=l1, l2=l2)
        u = [u1, u2, q3_up, -(u1+u2)]
        move_arm(sim, q=q, u=u)
        sim.step()

#~~~~~~~~~~~~~~~~~~~~~~~~~

# Stop the simulation
sim.stopSimulation()
sim.setStepping(False)
print('Stopped')

# Remove MTB model
sim.removeModel(robot)
