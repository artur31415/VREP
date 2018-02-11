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
    
        -- Put some initialization code here
        isConsoleActive = false
        --filePath0 = "C:\\Users\\ARTUR\\Documents\\PROJECTS!!\\kalman filter\\ENCODER_DATA.txt"
        --file_ENCODER = io.open(filePath0, "w")
        Sonar = simGetObjectHandle('Sonar')
    
        my_ui=simGetUIHandle('Odometry')
        simSetUIButtonLabel(my_ui,0,'Odometry')
    
        Robot_base = simGetObjectHandle('Robo_Movel')
        
        ANN_UI=simGetUIHandle('ANN')
        simSetUIButtonLabel(ANN_UI,0,'ANN Data')
    
        POS_UI=simGetUIHandle('MyPosition')    
        simSetUIButtonLabel(POS_UI,0,'Position')
    
        --Console2 = simAuxiliaryConsoleOpen('Debug_Encoders',100,0)
    
        LeftMotor=simGetObjectHandle('MotorEsquerdoDinamico')
        RightMotor=simGetObjectHandle('MotorDireitoDinamico')
        NominalVelocity = 1
        LeftVelocity = 0
        RightVelocity = 0
    
        Angule = {0.0,0.0}
        Distance = {0.0,0.0}
        DEG2RAD = math.pi/180
        RAD2DEG = 180/math.pi
        R = 0.034979108--0.0349--0.034184
        Last_IMU_Measure = 0.0
        Last_Acc_Pos = {0.0,0.0,0.0}
        Acc_Pos = {0.0,0.0,0.0}
        lastT = 0.0
        mypos = {0.0,0.0}
    
        IMU_ACC = {}
        IMU_GYRO = {}
        
        d = 0.066950--0.065468--0.07*0.96524
        ToTurn = false
        MyOrientation = 0.0
        square_counter = 0
        Direction = 'CCW' -- 'CW'>clockwise, 'CCW'>conter clockwise
        --simSetScriptSimulationParameter(sim_handle_all,'countL',0)
        --simSetScriptSimulationParameter(sim_handle_all,'countR',0)
        SensorInterrupt = simGetScriptHandle('Encoder_int') 
        CounterR = simGetScriptSimulationParameter(SensorInterrupt,'countR')
        CounterL = simGetScriptSimulationParameter(SensorInterrupt,'countL')
        MyPosition = {simGetObjectPosition(Robot_base,-1)[1],simGetObjectPosition(Robot_base,-1)[2],0.0,0.0,0.0,0.0}
    --//////////
    Angular_vel = {}
        Angular_vel[1] = simGetScriptSimulationParameter(SensorInterrupt,'AvelR')
        Angular_vel[2] = simGetScriptSimulationParameter(SensorInterrupt,'AvelL')
        status = 'stop'
    WallSensors = {simGetObjectHandle('Left'), --1
                        simGetObjectHandle('Middle'), --2
                        simGetObjectHandle('Right')} --5
    
    Wheel_ang = {0.0,0.0}
    Encoder_ticks = {0.0,0.0}
    Last_Counter = {0.0,0.0}
    last_timer0 = 0
    Ts = 0.3
    PPRev = 40
    NumOfTurns = 0
    --///////////////////////////ANN_VARIABLES_BEGIN//////////////////////////////////////
    
    Ni = 3
    Nh = 3
    No = 2
    W_ih = {{-4.56849407930386, 7.02170281280843, -3.45119950617347}, 
            {7.94924167076544, -3.12035660182684, -6.58833134559276}, 
            {5.51056830065626, 0.75105932982001, 0.271249224316448}}
    
    
    B_h = {-6.14993881651546, 0.489685106887923, 6.64064192399269}
    
    W_ho = {{-16.1062934870948, -0.823047509002572}, {5.67738246105603, -8.88197946396522}, {6.99841046663795, 15.6262773545691}}
    
    
    --W_ho = {{0, 0}, {0, 0}, {0, 0}}
    B_o = {2.09610544293644, 1.25917858298974}
    
    h_out = {0.0, 0.0, 0.0}
    Outputs = {0.0, 0.0}
    WallSensorsData = {0.0, 0.0, 0.0}
    
    --control Varaibles
    LastTurnAng = 0.0
    LastAng = 0.0
    IsToRotate = false
    Action = ' '
    LastTurnCounter = 0.0
    SonarDist = 0
    --////////////////////ANN_VARIABLES_END//////////////////////////////////////
    --/////////////////////////////////MOTOR_FUNCTIONS_BEGIN///////////////////////////////////////////
    function DifDrive(a, b)
    status = 'DifDrive'
    a = a or NominalVelocity
    b = b or NominalVelocity
    LeftVelocity = a
    RightVelocity = b
    simSetJointTargetVelocity(LeftMotor,LeftVelocity)
    simSetJointTargetVelocity(RightMotor,RightVelocity)
    end
    
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
    --/////////////////////////////////MOTOR_FUNCTIONS_END///////////////////////////////////////////
    --/////////////////////////////////ENCODER_FUNCTIONS_BEGIN///////////////////////////////////////////
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
    Distance[1] = Angule[1]*temp
    Distance[2] = Angule[2]*temp
    end
    
    function Reset()
    simSetScriptSimulationParameter(SensorInterrupt,'countL',0)
    simSetScriptSimulationParameter(SensorInterrupt,'countR',0)
    CounterR = simGetScriptSimulationParameter(SensorInterrupt,'countR')
    CounterL = simGetScriptSimulationParameter(SensorInterrupt,'countL')
    end
    
    function Encoder()
    Angular_vel = {simGetScriptSimulationParameter(SensorInterrupt,'AvelR') or 0, simGetScriptSimulationParameter(SensorInterrupt,'AvelL')or 0}
    
    CounterR = simGetScriptSimulationParameter(SensorInterrupt,'countR')
    CounterL = simGetScriptSimulationParameter(SensorInterrupt,'countL')
    Ang()
    Dis()
    Position()
    simSetUIButtonLabel(POS_UI,1,string.format('x = %.4f',MyPosition[1]))
    simSetUIButtonLabel(POS_UI,2,string.format('y = %.4f',MyPosition[2]))
    simSetUIButtonLabel(POS_UI,3,string.format('theda = %.4f',MyPosition[4]))
    
    end
    --/////////////////////////////////ENCODER_FUNCTIONS_END///////////////////////////////////////////
    
    
    function Position()
    --local euler = simGetObjectOrientation(modelBase,-1)
    local deg  = {0.0,0.0}
    vel = {Angular_vel[1] or 0, Angular_vel[2] or 0}
            if(status == 'forward') then
            vel[1] = Angular_vel[1] or 0
            vel[2] = Angular_vel[2] or 0
                if(vel[1] < 0 or vel[2] < 0) then
                vel[1] = 0
                vel[2] = 0
                end
            elseif(status == 'right_axis') then
            vel[1] = - Angular_vel[1] or 0
            vel[2] = Angular_vel[2] or 0
            elseif(status == 'left_axis') then
            vel[1] = Angular_vel[1] or 0
            vel[2] = - Angular_vel[2] or 0
            end
    
    if(MyPosition[5] < 0 or MyPosition[6] < 0) then
        MyPosition[5] = 0
        MyPosition[6] = 0
    end
    
        if(simGetSimulationTime() - last_timer0 >= Ts) then
            last_timer0 = simGetSimulationTime()
            MyPosition[5] = MyPosition[5] + Angular_vel[1]*Ts*RAD2DEG
            MyPosition[6] = MyPosition[6] + Angular_vel[2]*Ts*RAD2DEG
    
        if((status == 'left_axis' or status == 'right_axis') or ToRotate) then
            MyPosition[3] = MyPosition[3] + (R/(2*d))*(vel[2] - vel[1])*Ts--  + C0[3] --(180 + 45)*DEG2RAD--euler[2]
            if(MyPosition[3] >= 2*math.pi or MyPosition[3] <= -2*math.pi) then
                MyPosition[3] = 0
                NumOfTurns = NumOfTurns + 1
            end
            MyPosition[4] = MyPosition[3]*RAD2DEG
        else
                
                --simAuxiliaryConsolePrint(Console,MyPosition[1]..'\n')
                MyPosition[1] = MyPosition[1] + math.cos(MyPosition[3])*(R/2)*(vel[1] + vel[2])*Ts-- + C0[1]--math.cos(MyPosition[3])*(R/2)*(Wheel_ang[1] + Wheel_ang[2])*DEG2RAD + C0[1]
                MyPosition[2] = MyPosition[2] + math.sin(MyPosition[3])*(R/2)*(vel[1] + vel[2])*Ts-- + C0[2]
        end
    end
    
    --simAuxiliaryConsolePrint(Console,string.format('PosEstimation[1] = %.4f',PosEstimation[1])..'\n'..string.format('PosEstimation[2] = %.4f',PosEstimation[2])..'\n-------------------------------------------------------\n')
    
    end
    
    
    function MyData()
        local T = simGetSimulationTime()
        local position=simGetObjectPosition(Robot_base,-1)
        local euler = simGetObjectOrientation(Robot_base,-1)
        --file_ENCODER:write(Angule[1]..' '..Angule[2]..' '..T..'\n')
    end
    
    
    function Update_UI()
    simSetUIButtonLabel(my_ui,1,string.format("AngL: %.2f",Angule[1]))
    simSetUIButtonLabel(my_ui,2,string.format("AngR: %.2f",Angule[2]))
    simSetUIButtonLabel(my_ui,3,string.format("DistL: %.4f",Distance[1]))
    simSetUIButtonLabel(my_ui,4,string.format("DistR: %.4f",Distance[2]))
    end
    
    --/////////////////////ANN_FUNTIONS_BEING//////////////////////////////////
    
    
    
    
    function GetSensorData()
    result1,SonarDist=simReadProximitySensor(Sonar)--UltraSonico
    if(SonarDist == nil) then
    SonarDist = 0
    end
    
    
    if(SonarDist > 0 and SonarDist <= 0.1) then
    WallSensorsData[2] = 1
    else
    WallSensorsData[2] = 0
    end
    
    
    WallSensorsData[1] = simReadVisionSensor(WallSensors[1]);
    
    WallSensorsData[3] = simReadVisionSensor(WallSensors[3]);
    end
    
    function Sigmoid(xVal)
        return (1.0 /(1.0 + math.exp(-xVal)))
    
    end
    
    function ANN()
        
        --then compute the net output
        --hidden layer output
        local soma1 = 0.0
        local soma2 = 0.0
    
        for j=1,Nh do
            soma1 = 0.0
            for i=1,Ni do
                soma1 = soma1 + WallSensorsData[i]*W_ih[i][j]
            end
            h_out[j] = Sigmoid(soma1 + B_h[j])
        end
    
        
        --output layer output
        for j=1,No do
            soma2 = 0.0
            for i=1,Nh do
                soma2 = soma2 + h_out[i]*W_ho[i][j]
            end
            Outputs[j] = Sigmoid(soma2 + B_o[j])
        end
    end
    
    function ANN_Control()
    --local vel = {0, 0}
    local aVel = 3
    
    --LastAng = 0.0
    --IsToRotate = false
    if(Action == 'L' or Action == 'R') then
    --then rotate here!!!
    
        --if(WallSensorsData[1] == 0 and WallSensorsData[2] == 0 and WallSensorsData[3] == 0) then
        --Action = 'F'
        --Forward(aVel)
        --end
    --LastTurnCounter = NumberOfTurns
    if(Action == 'L') then
    LastTurnAng = LastAng + 360*(NumOfTurns - LastTurnCounter)
    elseif(Action == 'R') then
    LastTurnAng = LastAng - 360*(NumOfTurns - LastTurnCounter)
    end
    
        if(math.abs(MyPosition[4] - LastTurnAng) >= 60) then
        Action = ' '
        end
    
    
    elseif(Action == 'B') then
    --Backwards logic here! turn a 180 deg
        if(MyPosition[4] - LastAng >= 160) then
        Action = ' '
        end
    else
        if(Outputs[1] >= 0.9 and Outputs[2] >= 0.9) then-- 1 1 Forward
            Action = 'F'
            Forward(aVel)
        elseif(Outputs[1] >= 0.9 and Outputs[2] <= 0.5) then-- 1 0 Left
            Action = 'L'
            Left_axis(aVel)
            LastAng = MyPosition[4]
            LastTurnCounter = NumOfTurns
        elseif(Outputs[1] <= 0.5 and Outputs[2] >= 0.9) then-- 0 1 Right
            Action = 'R'
            Right_axis(aVel)
            LastAng = MyPosition[4]
            LastTurnCounter = NumOfTurns
        elseif(Outputs[1] <= 0.5 and Outputs[2] <= 0.5) then-- 0 0 Backwards
            Action = 'B'
            LastAng = MyPosition[4]
        end
    end
        --if(Outputs[1] >= 0.9) then
        --    vel[1] = aVel
        --else
        --    vel[1] = - aVel
        --end
    
        --if(Outputs[2] >= 0.9) then
        --    vel[2] = aVel
        --else
        --    vel[2] = - aVel
        --end
    
        --DifDrive(vel[1], vel[2])
    end
    
    function UpdateAnnGUI()
    simSetUIButtonLabel(ANN_UI,1,'Sensor[1] = '..WallSensorsData[1])
    simSetUIButtonLabel(ANN_UI,2,'Sensor[2] = '..WallSensorsData[2])
    simSetUIButtonLabel(ANN_UI,3,'Sensor[3] = '..WallSensorsData[3])
    simSetUIButtonLabel(ANN_UI,4,string.format("Outputs[1] = %.3f",Outputs[1]))
    simSetUIButtonLabel(ANN_UI,5,string.format("Outputs[2] = %.3f",Outputs[2]))--Action == 'R'
    simSetUIButtonLabel(ANN_UI,6,string.format("Action = %s",Action))--Action == 'R'
    simSetUIButtonLabel(ANN_UI,7,string.format("Angle = %.3f",MyPosition[4] - LastAng))--Action == 'R'
    simSetUIButtonLabel(ANN_UI,8,string.format("SonarDist = %.3f",SonarDist))
    end
    --/////////////////////ANN_FUNCTIONS_END/////////////////////////////////
    --/////////////////////END FUNCTIONS///////////////////////////
    end
    
    simHandleChildScripts(sim_call_type)
    
    -- Put your main code here
    --Forward()
    Encoder()
    GetSensorData()
    ANN()
    ANN_Control()
    UpdateAnnGUI()
    
    --Right_axis()
    Update_UI()
    
    
    if(simGetSimulationTime() - lastT >= 1) then
    lastT = simGetSimulationTime()
    end
    
    --simAuxiliaryConsolePrint(Console2,'A['..dis..'] = '..Model(0,{0,dis})[1]..'\n'..'----------------------'..'\n')
    
    
    
    if (simGetSimulationState()==sim_simulation_advancing_lastbeforestop) then
    
        -- Put some restoration code here
        --file_ENCODER:close()
    end
 
 
------------------------------------------------------------------------------ 
-- Following few lines automatically added by V-REP to guarantee compatibility 
-- with V-REP 3.1.3 and later: 
end 
------------------------------------------------------------------------------ 
