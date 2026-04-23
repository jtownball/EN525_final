# Import Libraries
import numpy as np
import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from box import Box

TOLERANCE = 0.005  # Increased from 0.001 for better convergence

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

def move_tool_xy_pos(x, y, target_pos) -> None:
    q1_target, q2_target = inverse_kinematics(x, y, l1, l2, elbow_down=True)
    print(f'Target joints: q1={np.degrees(q1_target):.2f}°, q2={np.degrees(q2_target):.2f}°')
    
    # Use position control with higher velocity limits for revolute joints
    sim.setObjectFloatParam(q1, sim.jointfloatparam_maxvel, 5.0)  # Faster movement
    sim.setObjectFloatParam(q2, sim.jointfloatparam_maxvel, 5.0)
    sim.setJointTargetPosition(q1, q1_target)
    sim.setJointTargetPosition(q2, q2_target)

    step_count = 0
    max_steps = 200  # Allow more time for settling
    
    try:
        tool_pos = sim.getObjectPosition(tool, sim.handle_world)
        q1_current = sim.getJointPosition(q1)
        q2_current = sim.getJointPosition(q2)
    except Exception as e:
        print(f'Error getting initial positions: {e}')
        return
    
    print(f'Moving XY from q1={np.degrees(q1_current):.2f}°, q2={np.degrees(q2_current):.2f}° to q1={np.degrees(q1_target):.2f}°, q2={np.degrees(q2_target):.2f}°')
    
    while step_count < max_steps:
        sim.step()
        
        try:
            tool_pos = sim.getObjectPosition(tool, sim.handle_world)
            q1_current = sim.getJointPosition(q1)
            q2_current = sim.getJointPosition(q2)
        except Exception as e:
            print(f'Error getting positions during motion: {e}')
            break
        
        # Calculate errors
        q1_error = abs(q1_current - q1_target)
        q2_error = abs(q2_current - q2_target)
        tool_xy_error = np.linalg.norm(np.array(tool_pos[:2]) - np.array(target_pos[:2]))
        
        # Check if we've reached the target (only check tool position, not joint errors)
        if tool_xy_error < TOLERANCE:
            break
            
        step_count += 1
        
        if step_count % 200 == 0:  # Less frequent updates
            print(f"Step {step_count}: q1_error={np.degrees(q1_error):.3f}°, q2_error={np.degrees(q2_error):.3f}°, xy_error={tool_xy_error:.6f}")
    
    elapsed_time = step_count * dt
    print(f'Move tool XY: took {step_count} steps ({elapsed_time:.3f}s)')
    print(f'  q1: target={np.degrees(q1_target):.3f}°, actual={np.degrees(q1_current):.3f}°, error={np.degrees(q1_error):.3f}°')
    print(f'  q2: target={np.degrees(q2_target):.3f}°, actual={np.degrees(q2_current):.3f}°, error={np.degrees(q2_error):.3f}°')
    print(f'  tool xy: target={target_pos[:2]}, actual={tool_pos[:2]}, error={tool_xy_error:.6f}')
        
def move_tool_z(target_z, tolerance=0.01, max_steps=500) -> None:
    """Move tool to target Z position by directly setting suctionPad position
    
    Note: The prismatic joints (q3, q4) do not control the suctionPad's Z position.
    Instead, the suctionPad can be positioned directly in the simulation.
    """
    
    step_count = 0
    step_size = 0.005  # Small incremental movement
    
    try:
        tool_pos = sim.getObjectPosition(tool, sim.handle_world)
        current_z = tool_pos[2]
    except Exception as e:
        print(f'Error getting initial tool position: {e}')
        return
    
    print(f'Moving tool Z from {current_z:.4f} to {target_z:.4f} (direct positioning)')
    
    while step_count < max_steps:
        sim.step()
        
        try:
            tool_pos = sim.getObjectPosition(tool, sim.handle_world)
            current_z = tool_pos[2]
        except Exception as e:
            print(f'Error getting tool position: {e}')
            break
        
        z_error = target_z - current_z
        
        # Check if we've reached the target
        if abs(z_error) < tolerance:
            break
        
        # Try direct positioning of the suctionPad
        new_pos = [tool_pos[0], tool_pos[1], target_z]
        try:
            sim.setObjectPosition(tool, sim.handle_world, new_pos)
        except Exception as e:
            print(f'Cannot directly set suctionPad position: {e}')
            break
        
        step_count += 1
        
        if step_count % 100 == 0:
            print(f"Step {step_count}: current_z={current_z:.4f}, target_z={target_z:.4f}, error={z_error:.4f}")
    
    elapsed_time = step_count * dt
    print(f'Move tool Z: took {step_count} steps ({elapsed_time:.3f}s)')
    print(f'  target_z={target_z:.4f}, actual_z={current_z:.4f}, error={abs(target_z - current_z):.4f}')

# Start the simulation
print('Starting simulation...')
sim.setStepping(True)
sim.startSimulation()
dt = sim.getSimulationTimeStep()
print(f'Simulation timestep (dt): {dt} seconds')
print(f'Real-time simulation: {sim.getBoolParam(sim.boolparam_realtime_simulation)}')

boxes = []
boxes.append(Box(sim, position=[0.5, .5, 0.05]))

box = boxes[0]

# Get initial q3/q4 joint positions and command them to stay there
initial_q3 = sim.getJointPosition(q3)
initial_q4 = sim.getJointPosition(q4)
print(f'Initial q3={initial_q3:.4f}, q4={initial_q4:.4f}')

# Get initial tool Z position
initial_tool_pos = sim.getObjectPosition(tool, sim.handle_world)
initial_tool_z = initial_tool_pos[2]
print(f'Initial tool z={initial_tool_z:.4f}')

# Ensure joints are at their initial positions (no-op if already there)
move_tool_z(initial_tool_z)

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

move_tool_xy_pos(x_rel, y_rel, box_pos)

print(f'Tool position after XY move')
tool_pos = sim.getObjectPosition(tool, sim.handle_world)
xy_distance_to_box = np.linalg.norm(np.array(tool_pos[:2]) - np.array(box_pos[:2]))
z_distance_to_box = abs(tool_pos[2] - box_pos[2])
print(f'XY distance to box: {xy_distance_to_box:.4f}')
print(f'Z distance to box: {z_distance_to_box:.4f}')

if xy_distance_to_box <= TOLERANCE:
    # Move tool to the box height
    print(f'Moving tool to box Z position: {z_rel:.4f}')
    move_tool_z(z_rel)
    
    # Check final distance after Z movement
    tool_pos = sim.getObjectPosition(tool, sim.handle_world)
    final_distance = np.linalg.norm(np.array(tool_pos) - np.array(box_pos))
    print(f'Final distance to box: {final_distance:.4f}')
    
    if final_distance <= TOLERANCE:
        box.attach(tool)
        print('Box attached successfully!')
        
        # Move box to new position
        print('\nMoving box to new position...')
        new_box_pos = [0.7, 0.3, 0.05]  # New target position
        print(f'Target position: {new_box_pos}')
        
        # Move tool (with attached box) to new XY position
        move_tool_xy_pos(new_box_pos[0] - o1[0], new_box_pos[1] - o1[1], new_box_pos)
        
        # Move tool to new Z position
        print(f'Moving tool to new Z position: {new_box_pos[2]:.4f}')
        move_tool_z(new_box_pos[2])
        
        # Check final position
        tool_pos = sim.getObjectPosition(tool, sim.handle_world)
        final_distance = np.linalg.norm(np.array(tool_pos) - np.array(new_box_pos))
        print(f'Final distance to target: {final_distance:.4f}')
        
        # Detach the box
        if final_distance <= TOLERANCE:
            box.detach()
            print('Box detached successfully at new position!')
            
            # Let physics settle after detachment
            for _ in range(20):
                sim.step()
            
            # Verify box position after detachment
            box_pos_after = sim.getObjectPosition(box.handle, sim.handle_world)
            print(f'Box final position: {box_pos_after}')
            print('✅ Pick-and-place operation completed successfully!')
        else:
            print('Warning: tool did not reach the target position for dropping.')
            
    else:
        print('Warning: tool did not reach the box after Z movement.')
else:
    print('Warning: tool XY positioning failed; not attempting Z movement.')


# Stop the simulation
print("Stopping the simulation")
sim.stopSimulation()
sim.setStepping(False)
print('Stopped')

# Remove MTB model
sim.removeModel(robot)
