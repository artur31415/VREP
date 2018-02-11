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
    
        Reading_Threshold = 0.3
        --Console1 = simAuxiliaryConsoleOpen('Debug_Encoders',100,0)
        LeftSensor=simGetObjectHandle('LeftEncoderSensor')
        RightSensor=simGetObjectHandle('RightEncoderSensor')
        
        right_to = simGetSimulationTime()
        right_dt = 0.0
        _AvelR = simGetScriptSimulationParameter(sim_handle_all,'AvelR') or 0.0
        _AvelL = simGetScriptSimulationParameter(sim_handle_all,'AvelL') or 0.0
        last_AvelR = 0.0
        _AacelR = simGetScriptSimulationParameter(sim_handle_all,'AacelR') or 0.0
        right_actual_state = false
        right_last_state = false
        right_cont = 0
        Sensor_stateR = false
        Sensor_counterR = simGetScriptSimulationParameter(sim_handle_all,'countR')
    
        left_actual_state = false
        left_last_state = false
        Sensor_stateL = false
        Sensor_counterL = simGetScriptSimulationParameter(sim_handle_all,'countL')
    
        Measure_threshold = 2 --number of measeres taken
        pos = 0.0442
    
    
        Ts = 0.3
        vel_LT = {simGetSimulationTime(), simGetSimulationTime()}
        last_right_count = 0
        last_left_count = 0
        WheelRadius = 0.0275
        DEG2RAD = math.pi/180
        RAD2DEG = 1/DEG2RAD
        PPRev = 40--100
        
    end
    
    simHandleChildScripts(sim_call_type)
    
    -- Put your main code here
    Sensor_counterR = simGetScriptSimulationParameter(sim_handle_all,'countR')
    Sensor_counterL = simGetScriptSimulationParameter(sim_handle_all,'countL')
    R1,readingL = simReadVisionSensor(LeftSensor)
    R2,readingR = simReadVisionSensor(RightSensor)
    
    if(Sensor_counterL[1] == 0) then -- in case the sensor was reseted
    last_left_count = 0
    end
    if(Sensor_counterR[1] == 0) then
    last_right_count = 0
    end
    
    if(readingL[9] >= Reading_Threshold) then
    left_actual_state = true
    else
    left_actual_state = false
    end
    
    if(readingR[9] >= Reading_Threshold) then
    right_actual_state = true
    else
    right_actual_state = false
    end
    
    if(left_actual_state ~= left_last_state) then
    left_last_state = left_actual_state
    Sensor_stateL = not Sensor_stateL
    Sensor_counterL[1] = Sensor_counterL[1] +1
    simSetScriptSimulationParameter(sim_handle_all,'countL',Sensor_counterL[1])
    end
    
    if(right_actual_state ~= right_last_state) then
    right_last_state = right_actual_state
    Sensor_stateR = not Sensor_stateR
    Sensor_counterR[1] = Sensor_counterR[1] +1
    simSetScriptSimulationParameter(sim_handle_all,'countR',Sensor_counterR[1])
    end
    
    --simAuxiliaryConsolePrint(Console1,'count_left = '..Sensor_counterL[1]..'\n'..'count_right = '..Sensor_counterR[1]..'\n-------------------------------\n')
    
    if(Sensor_counterR[1] - last_right_count ~= 0 and simGetSimulationTime() - vel_LT[2] >= Ts) then
        vel_LT[2] = simGetSimulationTime()
        local local_ang = (Sensor_counterR[1] - last_right_count)*(math.pi*2)/(PPRev)*RAD2DEG
        _AvelR = (Sensor_counterR[1] - last_right_count)*(math.pi*2)/(PPRev*Ts)--rad/s
        last_right_count = Sensor_counterR[1]
        local temp = _AvelR*WheelRadius*simGetSimulationTime()
        local temp2 = _AvelR*RAD2DEG*simGetSimulationTime()
        --simAuxiliaryConsolePrint(Console1,'local_angL = '..(local_ang)..' °\n-------------------------------\n')
        simSetScriptSimulationParameter(sim_handle_all,'AvelR',_AvelR)
        --simSetScriptSimulationParameter(sim_handle_all,'AacelR',_AacelR)
    end
    if(Sensor_counterL[1] - last_left_count ~= 0 and simGetSimulationTime() - vel_LT[1] >= Ts) then
        vel_LT[1] = simGetSimulationTime()
        local local_ang = (Sensor_counterL[1] - last_left_count)*(math.pi*2)/(PPRev)*RAD2DEG
        _AvelL = (Sensor_counterL[1] - last_left_count)*(math.pi*2)/(PPRev*Ts)--rad/s
        last_left_count = Sensor_counterL[1]
        local temp = _AvelR*WheelRadius*simGetSimulationTime()
        local temp2 = _AvelR*RAD2DEG*simGetSimulationTime()
        --simAuxiliaryConsolePrint(Console1,'local_angR = '..(local_ang)..' °\n-------------------------------\n')
        simSetScriptSimulationParameter(sim_handle_all,'AvelL',_AvelL)
        --simSetScriptSimulationParameter(sim_handle_all,'AacelR',_AacelR)
    end
    
    
    
    if (simGetSimulationState()==sim_simulation_advancing_lastbeforestop) then
    
        -- Put some restoration code here
    
    end
 
 
------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
end 
------------------------------------------------------------------------------ 
