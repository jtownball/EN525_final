# Box class for CoppeliaSim simulation
import numpy as np

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