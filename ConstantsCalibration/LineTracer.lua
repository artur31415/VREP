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
    
        filePath = "C:\\Users\\ARTUR\\Documents\\PROJECTS!!\\kalman filter\\Data\\TruePos.txt"
        Limitline = "C:\\Users\\ARTUR\\Documents\\PROJECTS!!\\kalman filter\\Data\\Encoder.txt"
        Sensorsline = "C:\\Users\\ARTUR\\Documents\\PROJECTS!!\\kalman filter\\Data\\IMU.txt"
        TruePos_data = io.open(filePath, "w")
        Encoder_data = io.open(Limitline, "w")
        IMU_data = io.open(Sensorsline, "w")
        --file:write(string.format("Orientation = %.4f",Orientation[3]).."\n")
        accelCommunicationTube=simTubeOpen(0,'accelerometerData'..simGetNameSuffix(nil),1) -- put this in the initialization phase
        gyroCommunicationTube=simTubeOpen(0,'gyroData'..simGetNameSuffix(nil),1) -- put this in the initialization phase
    
    
    
    
    
    
        modelBase = simGetObjectHandle('LineTracer')
        ui=simGetUIHandle('Sensors_Data')
        UI_Pos=simGetUIHandle('MyPosition')
    
    
        simSetUIButtonLabel(ui,0,'Sensors Output')
        simSetUIButtonLabel(UI_Pos,0,'Estimated Pose')
    
    
        SonarSensor=simGetObjectHandle('Sonar')
    
        --Console = simAuxiliaryConsoleOpen('Debug',100,0)
        LeftMotor=simGetObjectHandle('DynamicLeftJoint')
        RightMotor=simGetObjectHandle('DynamicRightJoint')
        NominalVelocity = 1--0.2/0.0275
        LeftVelocity = 0.0
        RightVelocity = 0.0
        SensorManager = simGetScriptHandle('EncoderManager')
        LineSensors = {simGetObjectHandle('LeftSensor_OUT'), --1
                        simGetObjectHandle('LeftSensor_IN'), --2
                        simGetObjectHandle('MiddleSensor'), --3
                        simGetObjectHandle('RightSensor_IN'), --4
                        simGetObjectHandle('RightSensor_OUT')} --5
    
        CounterR = simGetScriptSimulationParameter(SensorManager,'countR')
        CounterL = simGetScriptSimulationParameter(SensorManager,'countL')
        Angular_vel = {}
        Angular_vel[1] = simGetScriptSimulationParameter(SensorManager,'AvelR')
        Angular_vel[2] = simGetScriptSimulationParameter(SensorManager,'AvelL')
    
        Ltime = simGetSimulationTime()
        RAD2DEG = 180/math.pi
        DEG2RAD = 1/RAD2DEG
        Angule = {0.0,0.0} --wheels angules (°)
        R = 0.0275 -- wheel radius
        Distance = {0.0,0.0} --distance moved by each wheel
        ToRotate = false
        d = 0.057184--0.0581 -- distance between wheel divided by 2
        RotationAngule = 90 --angule to rotate 
        NominalAngule = 7.0--5.0--20.0
        kappa = 0.0
    
        alpha = 19.724--°
        zeta = 10.1584--°
        gama = 0.0--°
        x = 0.023--m
        y = {0.0,0.0,0.0,0.0,0.0}
        OError = 0.0--°
        Error_threshold = 0.2--0.45
        theda = 0.0--°
        TargetAngule = {0.0,0.0}
        inside_sensor_counter = 0.0;
        iteration_counter = 0.0;
        percentage = {0.0,0.0,0.0,0.0}
        count = 0.0
        Ltime = simGetSimulationTime()
    
        Map = {}
        MyPosition = {simGetObjectPosition(modelBase,-1)[1],simGetObjectPosition(modelBase,-1)[2],0.0,0.0,0.0,0.0}
        C0 = {0.0,0.0,0.0}
        Wheel_ang = {0.0,0.0}
        Encoder_ticks = {0.0,0.0}
        Last_Counter = {0.0,0.0}
        status = 'stop'
    
        Ts = 0.3
        last_timer0 = 0
        Accelerometer = {0.0,0.0,0.0}
        Gyroscope = {0.0,0.0,0.0}
        PosEstimation = {0.0443,0.0}
        Space = {-0.0443,0.0,0.0}
        vel = {0.0, 0.0}
    
        PPRev = 40--100
        NumOfTurns = 0
        detectedPoint = {0.0,0,0,0.0}
    
        direction = 'CCW'--'CW'/'CCW'
    --file:write(Ltime..' '..(OError*25)..'\n')
    --Sensors_line:write((19.724)..' '..(10.1584)..' '..(0)..' '..(-10.1584)..' '..(-19.724)..'\n')
    --max_and_min:write('<5° s1 s0 >5°'..'\n')
    --/////////////////////////////////////////FUNCTIONS_BEGIN///////////////////////
    --////////////////////////////////////////////////////////////////////////////// 
    
    --/////////////////////////////////////////MODEL_FUNCTIONS_BEGIN//////////////////////////////////////////  
    function IMU()
    local data1=simTubeRead(accelCommunicationTube)
        if (data1) then
             Accelerometer=simUnpackFloatTable(data1)
        end
    local data2=simTubeRead(gyroCommunicationTube)
         if (data2) then
             Gyroscope=simUnpackFloatTable(data2)
         end
    end
    
    function GetData()
    local pos = simGetObjectPosition(modelBase,-1)
    local euler = simGetObjectOrientation(modelBase,-1)
    IMU_data:write(simGetSimulationTime()..' '..Accelerometer[1]..' '..Accelerometer[2]..' '..Accelerometer[3]..' '..Gyroscope[1]..' '..Gyroscope[2]..' '..Gyroscope[3]..'\n')
    Encoder_data:write(simGetSimulationTime()..' '..vel[1]..' '..vel[2]..' '..MyPosition[1]..' '..MyPosition[2]..' '..MyPosition[3]..'\n')
    TruePos_data:write(simGetSimulationTime()..' '..pos[1]..' '..pos[2]..' '..pos[3]..' '..euler[1]..' '..euler[2]..' '..euler[3]..'\n')
    end
    
    function Position()
    --local euler = simGetObjectOrientation(modelBase,-1)
    local deg  = {0.0,0.0}
    vel = {Angular_vel[1][1] or 0, Angular_vel[2][1] or 0}
            if(status == 'forward') then
            vel[1] = Angular_vel[1][1] or 0
            vel[2] = Angular_vel[2][1] or 0
                if(vel[1] < 0 or vel[2] < 0) then
                vel[1] = 0
                vel[2] = 0
                end
            elseif(status == 'right_axis') then
            vel[1] = - Angular_vel[1][1] or 0
            vel[2] = Angular_vel[2][1] or 0
            elseif(status == 'left_axis') then
            vel[1] = Angular_vel[1][1] or 0
            vel[2] = - Angular_vel[2][1] or 0
            end
    
    if(MyPosition[5] < 0 or MyPosition[6] < 0) then
        MyPosition[5] = 0
        MyPosition[6] = 0
        end
    
    if(simGetSimulationTime() - last_timer0 >= Ts) then
    
            last_timer0 = simGetSimulationTime()
        
    
        MyPosition[5] = MyPosition[5] + Angular_vel[1][1]*Ts*RAD2DEG
        MyPosition[6] = MyPosition[6] + Angular_vel[2][1]*Ts*RAD2DEG
    
    
        if(ToRotate) then
            
            MyPosition[3] = MyPosition[3] + (R/(2*d))*(vel[2] - vel[1])*Ts--  + C0[3] --(180 + 45)*DEG2RAD--euler[2]
            if(MyPosition[3] >= 2*math.pi or MyPosition[3] <= -2*math.pi) then
            MyPosition[3] = 0
            end
            MyPosition[4] = MyPosition[3]*RAD2DEG
        else
                
                --simAuxiliaryConsolePrint(Console,MyPosition[1]..'\n')
                MyPosition[1] = MyPosition[1] + math.cos(MyPosition[3])*(R/2)*(vel[1] + vel[2])*Ts-- + C0[1]--math.cos(MyPosition[3])*(R/2)*(Wheel_ang[1] + Wheel_ang[2])*DEG2RAD + C0[1]
                MyPosition[2] = MyPosition[2] + math.sin(MyPosition[3])*(R/2)*(vel[1] + vel[2])*Ts-- + C0[2]
                
                
                Space[3] = 2*Space[2] - Space[1] + Accelerometer[1]*Ts*Ts
                Space[1] = Space[2]
                Space[2] = Space[3]
                PosEstimation[1] = Space[3]--PosEstimation[1] + Accelerometer[1]*Ts*Ts
                PosEstimation[2] = PosEstimation[2] + Accelerometer[2]*Ts*Ts
        end
    end
    
    --simAuxiliaryConsolePrint(Console,string.format('PosEstimation[1] = %.4f',PosEstimation[1])..'\n'..string.format('PosEstimation[2] = %.4f',PosEstimation[2])..'\n-------------------------------------------------------\n')
    
    end
    
    
    
    function Model(type,var) -- kinematics and dynamics model
    var = var or {0,0}
    local temp1 = {0.0,0.0}
    if(type == 0) then --kinematics model
        if(var[1] == 0) then --calculate the wheels angule to a translation with a distance var[2]
        temp1[1] = (var[2]/R)*RAD2DEG
        temp1[2] = (var[2]/R)*RAD2DEG
        elseif(var[1] == 1) then --calculate the wheels angule to a rotation with an angule var[2]
        temp1[1] = (var[2]*d)/R
        temp1[2] = (var[2]*d)/R
        elseif(var[1] == 2) then --calculate the wheels angule to a rotation with an angule var[2]
        temp1[1] = NominalAngule - 2*(var[2]*d)/R
        temp1[2] = NominalAngule
        elseif(var[1] == 3) then --calculate the wheels angule to a rotation with an angule var[2]
        temp1[1] = NominalAngule
        temp1[2] = 2*(var[2]*d)/R + NominalAngule
        end
    else --dynamics model
    end
    
    return temp1
    end
    --/////////////////////////////////////////MODEL_FUNCTIONS_END/////////////////////////////////////////////////////////// 
    
    --/////////////////////////////////////////ENCODER_FUNCTIONS_BEGIN/////////////////////////////////////////////////////////////
    function Ang()
    if(not ToRotate) then
        if(CounterR and CounterR >= Last_Counter[1]) then
        Encoder_ticks[1] = Encoder_ticks[1] + (CounterR - Last_Counter[1])
        Last_Counter[1] = CounterR
        end
        if(CounterL and CounterL >= Last_Counter[2]) then
        Encoder_ticks[2] = Encoder_ticks[2] + (CounterL - Last_Counter[2])
        Last_Counter[2] = CounterL
        end
    end
    
    Angule[1] = CounterR*(360/PPRev) or 0
    Angule[2] = CounterL*(360/PPRev) or 0
    Wheel_ang[1] = Encoder_ticks[1]*(360/PPRev)
    Wheel_ang[2] = Encoder_ticks[2]*(360/PPRev)
    end
    function Dis()
    local temp = DEG2RAD*R
    Distance[1] = MyPosition[5]*temp--Angule[1]*temp
    Distance[2] = MyPosition[6]*temp--Angule[2]*temp
    end
    function Reset()
    simSetScriptSimulationParameter(sim_handle_all,'countL',0)
    simSetScriptSimulationParameter(sim_handle_all,'countR',0)
    CounterR = simGetScriptSimulationParameter(SensorManager,'countR')
    CounterL = simGetScriptSimulationParameter(SensorManager,'countL')
    Last_Counter[1] = CounterR
    Last_Counter[2] = CounterL
    
    MyPosition[5] = 0
    MyPosition[6] = 0
    end
    function Encoder()
    Angular_vel = {simGetScriptSimulationParameter(sim_handle_all,'AvelR') or 0, simGetScriptSimulationParameter(sim_handle_all,'AvelL')or 0}
    
    CounterR = simGetScriptSimulationParameter(SensorManager,'countR')
    CounterL = simGetScriptSimulationParameter(SensorManager,'countL')
    Ang()
    Dis()
    Position()
    
    simSetUIButtonLabel(UI_Pos,1,string.format('x = %.4f',MyPosition[1]))
    simSetUIButtonLabel(UI_Pos,2,string.format('y = %.4f',MyPosition[2]))
    simSetUIButtonLabel(UI_Pos,3,string.format('theda = %.4f',MyPosition[4]))
    
    
    --simAuxiliaryConsolePrint(Console,string.format('y = %.4f',MyPosition[2])..'\n')
    --simAuxiliaryConsolePrint(Console,string.format('theda = %.4f',MyPosition[4])..'\n----------------------------------------------------\n')
    
    
    end
    --///////////////////////////////////////////ENCODER_FUNCTIONS_END///////////////////////////////////////////////////////////////////////
    
    --/////////////////////////////////////////MOTOR_FUNCTIONS_BEGIN/////////////////////////////////////////////////////////////////////////// 
    function Forward(a)
    status = 'forward'
    a = a or NominalVelocity
    LeftVelocity = a
    RightVelocity = a
    simSetJointTargetVelocity(LeftMotor,LeftVelocity)
    simSetJointTargetVelocity(RightMotor,RightVelocity)
    end
    function Backwards(a)
    status = 'backwards'
    a = a or NominalVelocity
    LeftVelocity = -a
    RightVelocity = -a
    simSetJointTargetVelocity(LeftMotor,LeftVelocity)
    simSetJointTargetVelocity(RightMotor,RightVelocity)
    end
    function Stop()
    status = 'stop'
    LeftVelocity = 0.0
    RightVelocity = 0.0
    simSetJointTargetVelocity(LeftMotor,LeftVelocity)
    simSetJointTargetVelocity(RightMotor,RightVelocity)
    end
    function Right(a,b)
    status = 'right'
    a = a or NominalVelocity
    b = b or 2
    LeftVelocity = a
    RightVelocity = a/b
    simSetJointTargetVelocity(LeftMotor,LeftVelocity)
    simSetJointTargetVelocity(RightMotor,RightVelocity)
    end
    function Left(a,b)
    status = 'left'
    a = a or NominalVelocity
    b = b or 2
    LeftVelocity = a/b
    RightVelocity = a
    simSetJointTargetVelocity(LeftMotor,LeftVelocity)
    simSetJointTargetVelocity(RightMotor,RightVelocity)
    end
    function Right_axis(a)
    status = 'right_axis'
    a = a or NominalVelocity
    LeftVelocity = -a
    RightVelocity = a
    simSetJointTargetVelocity(LeftMotor,LeftVelocity)
    simSetJointTargetVelocity(RightMotor,RightVelocity)
    end
    function Left_axis(a)
    status = 'left_axis'
    a = a or NominalVelocity
    LeftVelocity = a
    RightVelocity = -a
    simSetJointTargetVelocity(LeftMotor,LeftVelocity)
    simSetJointTargetVelocity(RightMotor,RightVelocity)
    end
    --/////////////////////////////////////////MOTOR_FUNCTIONS_END/////////////////////////////////////////////////////////////////////////// 
    
    --/////////////////////////////////////////TRAJECTORY_FUNCTIONS_BEGIN/////////////////////////////////////////////////////////////////////////// 
    function Move_a_square(L)
    L = L or 0.5 --the side of the square
        if(NumOfTurns >= 4) then
        Stop()
        elseif(not ToRotate) then
            if(Distance[1] >= L or Distance[2] >= L) then
            Stop()
            Reset()
            ToRotate = true
            Angule[1] = 0
            Angule[2] = 0
            NumOfTurns = NumOfTurns + 1
            else
            Forward()
            end
        elseif(ToRotate) then
            if(math.abs(MyPosition[4]) >= 90*NumOfTurns) then--if(MyPosition[5] >= Model(0,{1,90})[1] or MyPosition[6] >= Model(0,{1,90})[2]) then
            
            Stop()
            Reset()
            ToRotate = false
            else
                if(direction == 'CCW') then
                Right_axis()
                else
                Left_axis()
                end
            end
        end
    end    
    
    
    --/////////////////////////////////////////TRAJECTORY_FUNCTIONS_END/////////////////////////////////////////////////////////////////////////// 
    
    --/////////////////////////////////////////KALMAN_FILTER_END///////////////////////////////////////////////////////////////////////////
    K_pose = {}
    K_Ts = 0
    K_G = {}
    K_H = {}
    K_V = {}
    K_I = {}
    
    function K_Model(aPose,aComand)
    local ePose = {}
    ePose[1] = aPose[1] + math.cos(aPose[3])*(Rr/2)*(aComand[1] + aComand[2])*K_Ts-- + Error1
    ePose[2] = aPose[2] + math.sin(aPose[3])*(Rr/2)*(aComand[1] + aComand[2])*K_Ts-- + Error1
    ePose[3] = aPose[3] + (Rr/(2*d))*(aComand[2] - aComand[1])*K_Ts-- + Error1
    
    return ePose
    end
    
    function K_Jacobian_of_Model(aPose,aComand)
    local Jac = {}
    Jac[1][1] = 1
    Jac[1][2] = 0
    Jac[1][3] = - math.sin(aPose[3])*(Rr/2)*(aComand[1] + aComand[2])*K_Ts
    Jac[1][4] = math.cos(aPose[3])*(Rr/2)
    Jac[1][5] = math.cos(aPose[3])*(Rr/2)
    
    Jac[2][1] = 0
    Jac[2][2] = 1
    Jac[2][3] = math.cos(aPose[3])*(Rr/2)*(aComand[1] + aComand[2])*K_Ts
    Jac[2][4] = math.sin(aPose[3])*(Rr/2)
    Jac[2][5] = math.sin(aPose[3])*(Rr/2)
    
    Jac[3][1] = 0
    Jac[3][2] = 0
    Jac[3][3] = 1
    Jac[3][4] = - Rr/(2*d)
    Jac[3][5] = Rr/(2*d)
    
    return Jac
    end
    
    function K_Jacobian_of_Observation()
    local ob = {{0, 0, 0},{0, 0, 0},{0, 0, 0}}
    ob[1][1] = 1/(K_Ts*K_Ts)
    ob[2][2] = 1/(K_Ts*K_Ts)
    ob[3][3] = 1/(K_Ts)
    return ob
    end
    
    function K_HO_Variance()--variance to 5x5
    local myVar = {{K_V[1][1], K_V[1][2], K_V[1][3], 0, 0},{K_V[2][1], K_V[2][2], K_V[2][3], 0, 0},{K_V[3][1], K_V[3][2], K_V[3][3], 0, 0},{0, 0, 0, 1, 0},{0, 0, 0, 0, 1}}
    return myVar
    end
    
    --/////////////////////////////////////////KALMAN_FILTER_END///////////////////////////////////////////////////////////////////////////
    --/////////////////////////////////////////FUNCTIONS_END///////////////////////
    --////////////////////////////////////////////////////////////////////////////// 
    end
    
    
    simHandleChildScripts(sim_call_type)
    
    -- Put your main code here
    IMU()
    Encoder() -- Encoder manager to get the encoder sensors data and extract both angules and velocities
    GetData()
    --simSetJointTargetVelocity
    --simSetJointForce
    --Forward()
    myPos = {}
    if(simGetSimulationTime() - Ltime >= 0.5) then
        Ltime = simGetSimulationTime()
        --Forward()
        --Backwards()
        Move_a_square(4)
        if(status == 'stop' and NumOfTurns >= 4) then
        simStopSimulation()
        end
        --Left()
        --result,distance,detectedPoint=simReadProximitySensor(SonarSensor)
        --if(not distance) then
        --distance = 0
        --else
        --distance = (distance + 0.05)
        --end
    
        --if(not detectedPoint) then
        --myPos[1] = -1
        --myPos[2] = -1
        --myPos[3] = -1
        --else
        --myPos = detectedPoint
        --end
    --a,b = simReadVisionSensor(SonarSensor)
    --R[i],D[i] = simReadVisionSensor(LineSensors[i])string.format('Analog out = %.4f',b[11])
    simSetUIButtonLabel(ui,1,string.format('Wheel_ang[1] = %.3f',Wheel_ang[1]))
    simSetUIButtonLabel(ui,2,string.format('Wheel_ang[2] = %.3f',Wheel_ang[2]))
    simSetUIButtonLabel(ui,3,string.format('AVel[1] = %.3f',Angular_vel[1][1]))
    simSetUIButtonLabel(ui,4,string.format('AVel[2] = %.3f',Angular_vel[2][1]))
    simSetUIButtonLabel(ui,5,'status = '..status)
    simSetUIButtonLabel(ui,6,string.format('vel->ang = %.4f',MyPosition[5]))
    --simAuxiliaryConsolePrint(Console,'b[11]'..b[11]..'\n'..'b[12]'..b[12]..'\n'..'b[13]'..b[13]..'\n'..'b[14]'..b[14]..'\n'..'b[15]'..b[15]..'\n----------------------------------------\n')
    end
    
    if (simGetSimulationState()==sim_simulation_advancing_lastbeforestop) then
    
        -- Put some restoration code here
    
    
    
        TruePos_data:close()
        Encoder_data:close()
        IMU_data:close()
    end
 
 
------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
end 
------------------------------------------------------------------------------ 
