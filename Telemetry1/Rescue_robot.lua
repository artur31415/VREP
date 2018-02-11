------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
if (sim_call_type==sim_childscriptcall_initialization) then 
    simSetScriptAttribute(sim_handle_self,sim_childscriptattribute_automaticcascadingcalls,false) 
end 
if (sim_call_type==sim_childscriptcall_cleanup) then 
 
end 
if (sim_call_type==sim_childscriptcall_sensing) then 
    simHandleChildScripts(sim_call_type) 
end 
if (sim_call_type==sim_childscriptcall_actuation) then 
    if not firstTimeHere93846738 then 
        firstTimeHere93846738=0 
    end 
    simSetScriptAttribute(sim_handle_self,sim_scriptattribute_executioncount,firstTimeHere93846738) 
    firstTimeHere93846738=firstTimeHere93846738+1 
 
------------------------------------------------------------------------------ 
 
 
if (simGetScriptExecutionCount()==0) then
    
    FrontTrack = simGetObjectHandle('FrontTrack')
    BackTrack = simGetObjectHandle('BackTrack')
    
    VelDx = 200
    
    VelVariables = simGetScriptHandle('Rescue_robot') 
    LeftSideVel = simGetScriptSimulationParameter(VelVariables, 'LeftSideVel')
    RightSideVel = simGetScriptSimulationParameter(VelVariables, 'RightSideVel')
        

    GripperState = simGetScriptSimulationParameter(VelVariables, 'GripperState')

    RobotOperationMode = simGetScriptSimulationParameter(VelVariables, 'OperationMode')
    --Joystick Variables
    
    ----------
    JoyVariables = simGetScriptHandle('SerialJoystickMasterController') 
    JoyCmd = simGetScriptSimulationParameter(JoyVariables, 'JoyCmd')

    MyConsole = simAuxiliaryConsoleOpen('DEBUG',100,0)
    
    
    function Joystick_GetCmd()
        JoyCmd = simGetScriptSimulationParameter(JoyVariables, 'JoyCmd')

        --Change operation Mode
        if(JoyCmd == 01) then
            if(RobotOperationMode == "MOVE") then
                RobotOperationMode = "ARM"
            else
                RobotOperationMode = "MOVE"
            end

            simSetScriptSimulationParameter(VelVariables, 'OperationMode', RobotOperationMode)
            simAuxiliaryConsolePrint(MyConsole, JoyCmd..'\n'..RobotOperationMode..'\n-----------------------------\n')
        end
        
        if(RobotOperationMode == "MOVE") then
            --movement buttons
            if(JoyCmd == 00) then
                LeftSideVel = 500
                RightSideVel = 500
            elseif(JoyCmd == 11) then
                LeftSideVel = 500 + VelDx
                RightSideVel = 500 + VelDx
            elseif(JoyCmd == 12) then
                LeftSideVel = 500 - VelDx
                RightSideVel = 500 + VelDx
            elseif(JoyCmd == 13) then
                LeftSideVel = 500 - VelDx
                RightSideVel = 500 - VelDx
            elseif(JoyCmd == 14) then
                LeftSideVel = 500 + VelDx
                RightSideVel = 500 - VelDx
            end
            --increase/decrease velocity(Circle, X)
            if(JoyCmd == 02) then
                VelDx = VelDx + 10
            elseif(JoyCmd == 03) then
                if(VelDx > 20) then
                    VelDx = VelDx - 10
                end
            end
        
            --control front track
            if(JoyCmd == 06) then
                simSetJointTargetVelocity(FrontTrack, 0.5)
            elseif(JoyCmd == 08) then
                simSetJointTargetVelocity(FrontTrack, -0.5)
            else
                simSetJointTargetVelocity(FrontTrack, 0)
            end
        
            --control back track
            if(JoyCmd == 05) then
                simSetJointTargetVelocity(BackTrack, -0.5)
            elseif(JoyCmd == 07) then
                simSetJointTargetVelocity(BackTrack, 0.5)
            else
                simSetJointTargetVelocity(BackTrack, 0)
            end

            simSetScriptSimulationParameter(VelVariables, 'LeftSideVel', LeftSideVel)
            simSetScriptSimulationParameter(VelVariables, 'RightSideVel', RightSideVel)
            
        else --ARM
            if(JoyCmd == 04) then
                if(GripperState == "CLOSED") then
                    GripperState = "OPEN"
                else
                    GripperState = "CLOSED"
                end
                simAuxiliaryConsolePrint(MyConsole, GripperState..'\n'..RobotOperationMode..'\n-----------------------------\n')
                simSetScriptSimulationParameter(VelVariables, 'GripperState', GripperState)
            end
        end
    end--end Joystick_GetCmd
    
    
    
    end
    
    
    simHandleChildScripts(sim_call_type)
    
    -- Put your main code here
    Joystick_GetCmd()
    --LeftSideVel = 1000 + VelDx
    --RightSideVel = 1000 + VelDx
    
    if (simGetSimulationState()==sim_simulation_advancing_lastbeforestop) then
    
        -- Put some restoration code here
    end
 
 
------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
end 
------------------------------------------------------------------------------ 
