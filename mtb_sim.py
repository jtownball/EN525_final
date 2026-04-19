# Import Libraries
import numpy as np
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from box import Box

TOLERANCE = 0.001

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

def inverse_kinematics(x, y, l1, l2, elbow_down=True):
    # Analytical SCARA IK using the paper's formulas.
    # θ2 = ±2 atan2( sqrt((l1+l2)^2 - r^2), sqrt(r^2 - (l1-l2)^2) )
    # θ1 = atan2(y, x) - atan2( l2 sin θ2, l1 + l2 cos θ2 )  (elbow down)
    # θ1 = atan2(y, x) + atan2( l2 sin θ2, l1 + l2 cos θ2 )  (elbow up)
    r = np.sqrt(x**2 + y**2)
    if r < abs(l1 - l2) or r > (l1 + l2):
        raise ValueError(f'Target ({x:.3f}, {y:.3f}) is out of reach for l1={l1:.3f}, l2={l2:.3f}')

    numerator = np.sqrt(max(0.0, (l1 + l2)**2 - r**2))
    denominator = np.sqrt(max(0.0, r**2 - (l1 - l2)**2))
    q2 = 2 * np.arctan2(numerator, denominator)
    if not elbow_down:
        q2 = -q2

    q1 = np.arctan2(y, x)
    if elbow_down:
        q1 -= np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2))
    else:
        q1 += np.arctan2(l2 * np.sin(q2), l1 + l2 * np.cos(q2))

    return q1, q2

def move_tool_xy_pos(x, y) -> None:
    q1_target, q2_target = inverse_kinematics(x, y, l1, l2, elbow_down=True)
    print(f'Target joints: q1={np.degrees(q1_target):.2f}°, q2={np.degrees(q2_target):.2f}°')
    sim.setObjectFloatParam(q1, sim.jointfloatparam_maxvel, 10)
    sim.setObjectFloatParam(q2, sim.jointfloatparam_maxvel, 10)
    sim.setJointTargetPosition(q1, q1_target)
    sim.setJointTargetPosition(q2, q2_target)

    step_count = 0
    max_steps = 2000
    tool_pos = sim.getObjectPosition(tool, sim.handle_world)
    while np.linalg.norm(np.array(tool_pos) - np.array(box_pos)) > TOLERANCE and step_count < max_steps:
        sim.step()
        tool_pos = sim.getObjectPosition(tool, sim.handle_world)
        step_count += 1
        
def move_tool_tip(q3_target, q4_target, tolerance=0.05, max_steps=200) -> None:
    sim.setObjectFloatParam(q3, sim.jointfloatparam_maxvel, 10)
    sim.setObjectFloatParam(q4, sim.jointfloatparam_maxvel, 10)
    sim.setJointTargetPosition(q3, q3_target)
    sim.setJointTargetPosition(q4, q4_target)

    step_count = 0
    tool_pos = sim.getObjectPosition(tool, sim.handle_world)
    while step_count < max_steps and abs(tool_pos[2] - q3_target) > tolerance:
        sim.step()
        tool_pos = sim.getObjectPosition(tool, sim.handle_world)
        step_count += 1

    print(f'Move tool tip to {q3_target:.4f}: took {step_count} steps, z={tool_pos[2]:.4f}')

# Start the simulation
print('Starting simulation...')
sim.setStepping(True)
sim.startSimulation()
dt = sim.getSimulationTimeStep()

boxes = []
boxes.append(Box(sim, position=[0.5, .5, 0.05]))

box = boxes[0]

# Get initial q3/q4 joint positions and command them to stay there
initial_q3 = sim.getJointPosition(q3)
initial_q4 = sim.getJointPosition(q4)
print(f'Initial q3={initial_q3:.4f}, q4={initial_q4:.4f}')
move_tool_tip(initial_q3, initial_q4)

# Step a bit to stabilize the tool at the initial height
for _ in range(50):
    sim.step()

# Get current box position
box_pos = sim.getObjectPosition(box.handle, sim.handle_world)
x, y, z = box_pos
print(f'Box position at step : {box_pos}')

# Compute relative to base
x_rel = x - o1[0]
y_rel = y - o1[1]
z_rel = z
print(f'Relative position: {x_rel}, {y_rel}, {z_rel}')

move_tool_xy_pos(x_rel,y_rel)
# move_tool_tip(z_rel)

print(f'Tool position after')
tool_pos = sim.getObjectPosition(tool, sim.handle_world)
distance_to_box = np.linalg.norm(np.array(tool_pos) - np.array(box_pos))
print(f'Distance to box: {distance_to_box:.4f}')
if distance_to_box <= TOLERANCE:
    # Move tool vertical joints to the box height
    move_tool_tip(z_rel, z_rel)
    box.attach(tool)
else:
    print('Warning: tool did not reach the box within max steps; not attaching.')


# Stop the simulation
sim.stopSimulation()
sim.setStepping(False)
print('Stopped')

# Remove MTB model
sim.removeModel(robot)
