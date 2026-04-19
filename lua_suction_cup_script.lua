sim=require'sim'
simMTB=require'simMTB'

sim=require('sim')
simMTB=require('simMTB')

function sysCall_init()
         
    s=sim.getObject('../Sensor')
    l=sim.getObject('../LoopClosureDummy1')
    l2=sim.getObject('../LoopClosureDummy2')
    b=sim.getObject('../BodyRespondable')
    sim.setLinkDummy(l,-1)
    sim.setObjectParent(l,b,true)
    m=sim.getObjectMatrix(l2)
    sim.setObjectMatrix(l,m)
    associatedRobotHandle=sim.getObject('::')
    
end

function sysCall_cleanup() 
    sim.setLinkDummy(l,-1)
    sim.setObjectParent(l,b,true)
    m=sim.getObjectMatrix(l2)
    sim.setObjectMatrix(l,m)
end 

function sysCall_actuation() 
    -- Set the active/inactive state (directly controlled by the robot program):

    active=false

    -- here, read the active flag from the robot server (i.e. to activate the suction cup):
    local serverHandlePacked = sim.readCustomDataBlock(associatedRobotHandle, 'tmpMTBSERVERHANDLE')
    if serverHandlePacked and #serverHandlePacked > 0 then
        local serverHandle = sim.unpackInt32Table(serverHandlePacked)[1]
        robotOutput = simMTB.getOutput(serverHandle)
        active = ((robotOutput[1] & 1) == 1)
    end
    
    parent=sim.getObjectParent(l)
    if (active==false) then
        if (parent~=b) then
            sim.setLinkDummy(l,-1)
            sim.setObjectParent(l,b,true)
            m=sim.getObjectMatrix(l2)
            sim.setObjectMatrix(l,m)
        end
    else
        if (parent==b) then
            -- Here we want to detect a respondable shape, and then connect to it with a force sensor (via a loop closure dummy dummy link)
            -- However most respondable shapes are set to "non-detectable", so "sim.readProximitySensor" or similar will not work.
            -- But "sim.checkProximitySensor" or similar will work (they don't check the "detectable" flags), but we have to go through all shape objects!
            index=0
            while true do
                shape=sim.getObjects(index,sim.sceneobject_shape)
                if (shape==-1) then
                    break
                end
                if (shape~=b) and (sim.checkProximitySensor(s,shape)==1) then
                    -- Ok, we found a shape that was detected. Is it respondable?
                    if (sim.getObjectInt32Param(shape,sim.shapeintparam_respondable)~=0) then
                        -- Ok, we connect to that shape:
                        sim.setObjectParent(l,shape,true)
                        sim.setLinkDummy(l,l2)
                        break
                    end
                end
                index=index+1
            end
        end
    end
end 
