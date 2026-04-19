sim=require'sim'
simUI=require'simUI'
simMTB=require'simMTB'

function sysCall_init()
    -- Get some object handles that are required later:
    robotHandle=sim.getObject('..')
    jointHandles={-1,-1,-1,-1}
    for i=1,4,1 do
        jointHandles[i]=sim.getObject('../axis',{index=i-1})
    end
    robotName=sim.getObjectAlias(sim.getObject('..'),1)
    collisionMessageID=-1
    compilErrorMessageID=-1
    dfltButProp=sim.buttonproperty_button+sim.buttonproperty_horizontallycentered+sim.buttonproperty_staydown+sim.buttonproperty_verticallycentered
    jointPositions={0,0,0,0}
    for i=1,4,1 do
        jointPositions[i]=sim.getJointPosition(jointHandles[i]) -- the initial joint positions
    end
    restarting=false
    cmdMessage=''
    robotProgramExecutionState=1 -- 0 is stopped, 1 is executing, 2 is paused
    -- Now start the server:
    serverHandle=startRobotServer(robotHandle,program,{0,0,0,0},{0.1,0.4})
end

program=[[REM ************************************************
REM This is a very very simple robot language EXAMPLE!
REM Following commands are supported (one command per line):
REM -"REM" starts a comment line
REM -"SETROTVEL v": sets the revolute joint velocity for next movements (in degrees/s)
REM -"SETLINVEL v": sets the prismatic joint velocity for next movements (in meters/s)
REM -"MOVE p1 p2 p3 p4": moves to joint positions (p1;p2;p3;p4) (in degrees or meter)
REM -"WAIT x": waits x miliseconds
REM -"SETBIT y": sets the bit at pos y in the output buffer
REM -"CLEARBIT y": clears the bit at pos y in the output buffer
REM -"IFBITGOTO y label": jumps to "label" position if bit at pos y is set in the input buffer
REM -"IFNBITGOTO y label": jumps to "label" position if bit at pos y is not set in the input buffer
REM -"GOTO label": jumps to "label" position
REM any not recognized word is considered to be a label
REM ************************************************
SETROTVEL 45
SETLINVEL 0.1
MOVE 0 0 0 0
PROGRAM_BEGIN_LABEL
MOVE 0 0 0.03 0
IFBITGOTO 1 LABEL1
SETBIT 1
LABEL1
WAIT 500
MOVE 0 0 0 0
WAIT 250
MOVE -160 -43.5 0 203.5
WAIT 250
MOVE -160 -43.5 0.03 203.5
CLEARBIT 1
WAIT 500
MOVE -160 -43.5 0 203.5
WAIT 250
MOVE 160 43.5 0 -203.5
WAIT 250
MOVE 160 43.5  0.03 -203.5
IFBITGOTO 1 LABEL2
SETBIT 1
LABEL2
WAIT 500
MOVE 160 43.5  0 -203.5
GOTO LABEL1
]]

-- ***********************************************************************************************************************
-- ***********************************************************************************************************************
-- ***********************************************************************************************************************
-- This script makes the link between the MTB robot and the simExtMtb.dll plugin. This script could be much
-- shorter and simple, if everything was taken care of in the plugin. The advantage of handling many things here
-- is that if something needs to be changed, the plugin doesn't require to be recompiled
-- Following commands are registered by the MTB plugin:
--
-- number serverHandle,string msg=simMTB.startServer(string serverExecutable,number connectionPort,string programData,table_4 jointValues,table_2 initialJointVelocity)
-- Starts a robot server, and compiles the program. If result>=0, the call was successful. Otherwise msg contains a compilation error message
--
-- boolean result=simMTB.stopServer(number serverHandle)
-- Stops a robot server
--
-- number result,string info=simMTB.step(number serverHandle,number deltaTime)
-- runs the robot language interpreter for deltaTime. if result==0, the program is running, if result==1 the program ended. Any other value means an error
--
-- table_4 jointValues=simMTB.getJoints(number serverHandle)
-- returns the values of the robot's 4 axis
--
-- table_4 outputValues=simMTB.getOutput(number serverHandle)
-- returns the 32 bits (4*8) of the robot's outputs
--
-- table_4 inputValues=simMTB.getInput(number serverHandle)
-- returns the 32 bits (4*8) of the robot's inputs
--
-- number result=simMTB.setInput(number serverHandle,table_4 inputValues)
-- writes the 32 bits (4*8) of the robot's inputs
--
-- boolean result=simMTB.connectInput(number inputServerHandle,number inputBitNumber,number outputServerHandle,number outputBitNumber,number connectionType)
-- connects a robot's output bit to another robot's input bit. If connectionType~=0 the connection line invertes the bit state
--
-- boolean result=simMTB.disconnectInput(number inputServerHandle,number inputBitNumber)
-- disconnects a connection previously done with simMTB.connectInput
--
-- ***********************************************************************************************************************
-- ***********************************************************************************************************************
-- ***********************************************************************************************************************

function displayUiIfNeeded()
    local objs=sim.getObjectSel()
    if objs~=nil and objs[#objs]==robotHandle then
        createUi()
    else
        removeUi()
    end
end

function createUi()
    if ui==nil then
        local xml = '<ui title="'..sim.getObjectAlias(robotHandle,1)..'" closeable="false" resizeable="false" activate="false" placement="relative" position="-50,50">'..[[
            <tabs>
                <tab title="Command">
                    <group layout="hbox" flat="true">
                        <button text="Start/Restart" on-click="start_callback" id="1"/>
                        <button text="Pause" on-click="pause_callback" id="2"/>
                        <button text="Stop" on-click="stop_callback" id="3"/>
                    </group>
                    <group layout="form" flat="true">
                        <label text="Command" />
                        <edit id="4" enabled="false"/>
                    </group>
                </tab>
                <tab title="Joints">
                    <group layout="form" flat="true">
                        <label text="Joint 1" />
                        <edit id="11" enabled="false"/>
                        <label text="Joint 2" />
                        <edit id="12" enabled="false"/>
                        <label text="Joint 3" />
                        <edit id="13" enabled="false"/>
                        <label text="Joint 4" />
                        <edit id="14" enabled="false"/>
                    </group>
                </tab>
                <tab title="Input">
                    <group layout="grid" flat="true">
                    <label text="" />
                    <label text="1" />
                    <label text="2" />
                    <label text="3" />
                    <label text="4" />
                    <label text="5" />
                    <label text="6" />
                    <label text="7" />
                    <label text="8" />
                    <br/>
                    <label text="+00" />
                    <checkbox text="" on-change="inputClicked_callback" id="101"/>
                    <checkbox text="" on-change="inputClicked_callback" id="102"/>
                    <checkbox text="" on-change="inputClicked_callback" id="103"/>
                    <checkbox text="" on-change="inputClicked_callback" id="104"/>
                    <checkbox text="" on-change="inputClicked_callback" id="105"/>
                    <checkbox text="" on-change="inputClicked_callback" id="106"/>
                    <checkbox text="" on-change="inputClicked_callback" id="107"/>
                    <checkbox text="" on-change="inputClicked_callback" id="108"/>
                    <br/>
                    <label text="+08" />
                    <checkbox text="" on-change="inputClicked_callback" id="109"/>
                    <checkbox text="" on-change="inputClicked_callback" id="110"/>
                    <checkbox text="" on-change="inputClicked_callback" id="111"/>
                    <checkbox text="" on-change="inputClicked_callback" id="112"/>
                    <checkbox text="" on-change="inputClicked_callback" id="113"/>
                    <checkbox text="" on-change="inputClicked_callback" id="114"/>
                    <checkbox text="" on-change="inputClicked_callback" id="115"/>
                    <checkbox text="" on-change="inputClicked_callback" id="116"/>
                    <br/>
                    <label text="+16" />
                    <checkbox text="" on-change="inputClicked_callback" id="117"/>
                    <checkbox text="" on-change="inputClicked_callback" id="118"/>
                    <checkbox text="" on-change="inputClicked_callback" id="119"/>
                    <checkbox text="" on-change="inputClicked_callback" id="120"/>
                    <checkbox text="" on-change="inputClicked_callback" id="121"/>
                    <checkbox text="" on-change="inputClicked_callback" id="122"/>
                    <checkbox text="" on-change="inputClicked_callback" id="123"/>
                    <checkbox text="" on-change="inputClicked_callback" id="124"/>
                    <br/>
                    <label text="+24" />
                    <checkbox text="" on-change="inputClicked_callback" id="125"/>
                    <checkbox text="" on-change="inputClicked_callback" id="126"/>
                    <checkbox text="" on-change="inputClicked_callback" id="127"/>
                    <checkbox text="" on-change="inputClicked_callback" id="128"/>
                    <checkbox text="" on-change="inputClicked_callback" id="129"/>
                    <checkbox text="" on-change="inputClicked_callback" id="130"/>
                    <checkbox text="" on-change="inputClicked_callback" id="131"/>
                    <checkbox text="" on-change="inputClicked_callback" id="132"/>
                    </group>
                </tab>
                <tab title="Output">
                    <group layout="grid" flat="true">
                    <label text="" />
                    <label text="1" />
                    <label text="2" />
                    <label text="3" />
                    <label text="4" />
                    <label text="5" />
                    <label text="6" />
                    <label text="7" />
                    <label text="8" />
                    <br/>
                    <label text="+00" />
                    <checkbox text="" enabled="false" id="201"/>
                    <checkbox text="" enabled="false" id="202"/>
                    <checkbox text="" enabled="false" id="203"/>
                    <checkbox text="" enabled="false" id="204"/>
                    <checkbox text="" enabled="false" id="205"/>
                    <checkbox text="" enabled="false" id="206"/>
                    <checkbox text="" enabled="false" id="207"/>
                    <checkbox text="" enabled="false" id="208"/>
                    <br/>
                    <label text="+08" />
                    <checkbox text="" enabled="false" id="209"/>
                    <checkbox text="" enabled="false" id="210"/>
                    <checkbox text="" enabled="false" id="211"/>
                    <checkbox text="" enabled="false" id="212"/>
                    <checkbox text="" enabled="false" id="213"/>
                    <checkbox text="" enabled="false" id="214"/>
                    <checkbox text="" enabled="false" id="215"/>
                    <checkbox text="" enabled="false" id="216"/>
                    <br/>
                    <label text="+16" />
                    <checkbox text="" enabled="false" id="217"/>
                    <checkbox text="" enabled="false" id="218"/>
                    <checkbox text="" enabled="false" id="219"/>
                    <checkbox text="" enabled="false" id="220"/>
                    <checkbox text="" enabled="false" id="221"/>
                    <checkbox text="" enabled="false" id="222"/>
                    <checkbox text="" enabled="false" id="223"/>
                    <checkbox text="" enabled="false" id="224"/>
                    <br/>
                    <label text="+24" />
                    <checkbox text="" enabled="false" id="225"/>
                    <checkbox text="" enabled="false" id="226"/>
                    <checkbox text="" enabled="false" id="227"/>
                    <checkbox text="" enabled="false" id="228"/>
                    <checkbox text="" enabled="false" id="229"/>
                    <checkbox text="" enabled="false" id="230"/>
                    <checkbox text="" enabled="false" id="231"/>
                    <checkbox text="" enabled="false" id="232"/>
                    </group>
                </tab>
            </tabs>
            </ui>
            ]]
        ui=simUI.create(xml)
    end
end

function removeUi()
    if ui then
        simUI.destroy(ui)
        ui=nil
    end
end

function updateUiIfNeeded()
    if ui then
        -- Update the main custom dialog:
        -- The "run" button:
        simUI.setEnabled(ui,1,robotProgramExecutionState~=1)
        -- The "pause" button:
        simUI.setEnabled(ui,2,robotProgramExecutionState==1)
        -- The "stop" button:
        simUI.setEnabled(ui,3,robotProgramExecutionState~=0)
        -- The command label:
        simUI.setEditValue(ui,4,cmdMessage)
        -- The joint labels:
        simUI.setEditValue(ui,11,string.format('%.2f',jointPositions[1]*180/math.pi))
        simUI.setEditValue(ui,12,string.format('%.2f',jointPositions[2]*180/math.pi))
        simUI.setEditValue(ui,13,string.format('%.4f',jointPositions[3]))
        simUI.setEditValue(ui,14,string.format('%.2f',jointPositions[4]*180/math.pi))
        
        -- Update (read and write) the IN/OUT custom dialog:
        for i=0,3,1 do
            for j=0,7,1 do
                if ((inputData[1+i]&(2^j))~=0) then
                    simUI.setCheckboxValue(ui,101+j+i*8,2)
                else
                    simUI.setCheckboxValue(ui,101+j+i*8,0)
                end
                if ((outputData[1+i]&(2^j))~=0) then
                    simUI.setCheckboxValue(ui,201+j+i*8,2)
                else
                    simUI.setCheckboxValue(ui,201+j+i*8,0)
                end
            end
        end
    end
end

startRobotServer=function(theRobotHandle,theProgram,initJoints,initVel)
    -- Find a (hopefully) free port:
    local portNb=sim.getInt32Param(sim.intparam_server_port_next)
    local portStart=sim.getInt32Param(sim.intparam_server_port_start)
    local portRange=sim.getInt32Param(sim.intparam_server_port_range)
    local newPortNb=portNb+1
    if (newPortNb>=portStart+portRange) then
        newPortNb=portStart
    end
    sim.setInt32Param(sim.intparam_server_port_next,newPortNb)
    -- Start the server:
    local serverHandle,errorMsg=simMTB.startServer('mtbServer',portNb,theProgram,initJoints,initVel)
    if serverHandle<0 then
        -- We have a problem. Display the message:
        sim.addLog(sim.verbosity_scripterrors,'Robot \''..robotName..'\' caused a program compilation error: '..errorMsg)
    end
    -- Now write the server handle to the robot object, so that other objects can access that handle too:
    sim.writeCustomDataBlock(theRobotHandle, 'tmpMTBSERVERHANDLE', sim.packInt32Table({serverHandle}))
    return serverHandle
end

function start_callback()
    -- "Run" was pressed
    if (robotProgramExecutionState==0) then
        robotProgramExecutionState=1
    else
        if (robotProgramExecutionState==2) then robotProgramExecutionState=1 end
    end
end

function pause_callback()
    -- "Pause" was pressed
    if (robotProgramExecutionState==1) then robotProgramExecutionState=2 end
end

function stop_callback()
    -- "Stop" was pressed
    if (robotProgramExecutionState~=0) then
        robotProgramExecutionState=0
        restarting=true
        cmdMessage=''
    end
end

function inputClicked_callback(ui,id,newVal)
    inputData[1+math.floor((id-101)/8)]=(inputData[1+math.floor((id-101)/8)]~(2^math.mod(id-101,8)))
    simMTB.setInput(serverHandle,inputData)
end


function sysCall_cleanup()
    sim.writeCustomDataBlock(robotHandle, 'tmpMTBSERVERHANDLE', nil)
    if serverHandle>=0 then
        simMTB.stopServer(serverHandle)
    end
    removeUi()
end 

function sysCall_actuation()
    displayUiIfNeeded()
    -- Following section is where the script is communicating with the extension module:
    -- #################################################################################
    if serverHandle>=0 then
        if (robotProgramExecutionState>0) then
            dt=sim.getSimulationTimeStep()
            if (robotProgramExecutionState==2) then dt=0 end -- When pausing, we simply set dt to zero!
    
            if (restarting) then
                -- Reset the robot and interpreter:
                simMTB.stopServer(serverHandle)
                serverHandle=startRobotServer(robotHandle,program,{0,0,0,0},{0.1,0.4})
                restarting=false
            else
                -- Handle the robot program (simMTB.run is a custom Lua command defined in the simExtMtb.dll extension module):
                result,cmdMessage,newJointPositions=simMTB.step(serverHandle,dt)
                if (result~=-1) then
                    -- program executes fine
                    -- Read the joint values and the robot's output (simMTB.getJoints and simMTB.getOutput are custom Lua command defined in the simExtMtb.dll extension module):
                    jointPositions=simMTB.getJoints(serverHandle)
                    outputData=simMTB.getOutput(serverHandle)
                    if (result==1) then
                        -- program end
                        robotProgramExecutionState=0
                        restarting=true
                        cmdMessage=''
                    end
                end
            end
        end
    
        -- Read the robot's input (simMTB.getInput is a custom Lua command defined in the simExtMtb.dll extension module):
        inputData=simMTB.getInput(serverHandle)
    end
    -- ---------------------------------------------------------------------------------
    
    -- Report the new joint positions to the MTB robot:
    for i=1,4,1 do
        sim.setJointPosition(jointHandles[i],jointPositions[i])
    end
    
    updateUiIfNeeded()
end 
