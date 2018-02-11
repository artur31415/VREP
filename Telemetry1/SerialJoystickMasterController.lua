-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

if (sim_call_type==sim_childscriptcall_initialization) then

    JoyVariables = simGetScriptHandle('SerialJoystickMasterController') 
    SerialPortName = simGetScriptSimulationParameter(JoyVariables, 'SerialPortName')
    BaudRate = simGetScriptSimulationParameter(JoyVariables, 'BaudRate')
    
    --Joystick Variables
    
    Serial = simSerialOpen(SerialPortName, 9600)
    JoyCmd = " "
    --MyConsole = simAuxiliaryConsoleOpen('DEBUG',100,0)
    
    
--//////////////////////////////////////////////////////////////////////////////////////
    function Joystick_GetSerialCmd()
        if(simSerialCheck(Serial) >= 2) then
            JoyCmd = simSerialRead(Serial, 2 ,false)
        
            if(JoyCmd == "10") then
            simStopSimulation()
            end
        
        simSetScriptSimulationParameter(JoyVariables, 'JoyCmd', JoyCmd)
        --simAuxiliaryConsolePrint(MyConsole, JoyCmd..'\n-----------------------------\n')
        end
    end
--//////////////////////////////////////////////////////////////////////////////////////////
end


if (sim_call_type==sim_childscriptcall_actuation) then

    -- Put your main ACTUATION code here

    Joystick_GetSerialCmd()

end


if (sim_call_type==sim_childscriptcall_sensing) then

    -- Put your main SENSING code here

end


if (sim_call_type==sim_childscriptcall_cleanup) then

    -- Put some restoration code here
    simSerialClose(Serial)
end