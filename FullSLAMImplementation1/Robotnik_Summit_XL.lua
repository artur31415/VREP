-- DO NOT WRITE CODE OUTSIDE OF THE if-then-end SECTIONS BELOW!! (unless the code is a function definition)

if (sim_call_type==sim_childscriptcall_initialization) then

RobotParameters = simGetScriptHandle('Robotnik_Summit_XL')
TimeSample = tonumber(simGetScriptSimulationParameter(RobotParameters, 'TimeSample'))

motorHandles={simGetObjectHandle('joint_front_left_wheel'),
              simGetObjectHandle('joint_front_right_wheel'),
              simGetObjectHandle('joint_back_right_wheel'),
              simGetObjectHandle('joint_back_left_wheel')}

------------------------SONAR PART---------------------------------------------
SonarSensors = {}
SonarManager = simGetObjectHandle('SonarSensorManager')
for k = 1, 11 do
    SonarSensors[k] = simGetObjectHandle('Ray'..(k-1))
end

SensorsState = {}
SensorsDistance = {}
SensorFramnePosition = {90, 90 - 18, 90 - 36, 90 - 54, 90 - 72, 0, 360 - (90 - 72), 360 - (90 - 54), 360 - (90 - 36), 360 - (90 - 18), 360 - 90}
SensorTheda = {}
Sd = {0.425 - 0.614, 0.275 - 0.425, 0.425 - 0.614, 0.275 - 0.425}
D = {}

---------------------------------------------------------------------------------
SLAM_Serial = simSerialOpen('COM1',9600)
LastSerialTime = 0

--simHandleProximitySensor
Image_filePath = "C:\\Users\\sigma\\Documents\\PROJECTS!!\\SLAM\\SimData\\CAMERA_DATA.txt"
RobotCamera = simGetObjectHandle('ControlCamera')

RobotHandle = simGetObjectHandle('Robotnik_Summit_XL')

-- motorHandles[1]=simGetObjectHandle('SummitXL_frontLeftMotor')
-- motorHandles[2]=simGetObjectHandle('SummitXL_frontRightMotor')
-- motorHandles[3]=simGetObjectHandle('SummitXL_rearRightMotor')
-- motorHandles[4]=simGetObjectHandle('SummitXL_rearLeftMotor')
Serial = simSerialOpen('COM3',9600)
JoyCmd = " " 
MyUi=simGetUIHandle('Robot')
simSetUIButtonLabel(MyUi, 0, "Robotinik data")



--Console = simAuxiliaryConsoleOpen('Debug_Robot',100,0)

RobotMotion = "Stop"
LRobotMotion = "Stop"


--ENCODER VARAIBLES
ABSParameters = {'M1_A', 'M2_A', 'M3_A', 'M4_A'}
IParameters = {'M1_IA', 'M2_IA', 'M3_IA', 'M4_IA'}
SensorManager = simGetScriptHandle('EncoderManager')
Ang = {0.0, 0.0, 0.0, 0.0}
ABSAng = {0.0, 0.0, 0.0, 0.0}
RAD2DEG = 180.0/math.pi
DEG2RAD = math.pi/180.0
StartPostion = {simGetObjectPosition(RobotHandle, -1)[1], simGetObjectPosition(RobotHandle, -1)[2], 0}
MyPosition = {StartPostion[1], StartPostion[2], 0.0, 0.0}
simMyPosition = {0.0, 0.0, 0.0, 0.0}
DistanceSonar = {0.0, 0.0, 0.0, 0.0}
NumOfTurns = 0
R = 0.11685
d = 0.234
IsFirst = true
ImageRead = false
LTime = 0
------------------------------------
function Stop()
RobotMotion = "Stop"
simSetJointTargetVelocity(motorHandles[1], 0)
simSetJointTargetVelocity(motorHandles[2], 0)
simSetJointTargetVelocity(motorHandles[3], 0)
simSetJointTargetVelocity(motorHandles[4], 0)
end
function Forward(vel)
vel = vel or 1
RobotMotion = "Forward"
simSetJointTargetVelocity(motorHandles[1], vel)
simSetJointTargetVelocity(motorHandles[2], vel)
simSetJointTargetVelocity(motorHandles[3], vel)
simSetJointTargetVelocity(motorHandles[4], vel)
end
function Backward(vel)
vel = vel or 1
RobotMotion = "Backward"
simSetJointTargetVelocity(motorHandles[1], -vel)
simSetJointTargetVelocity(motorHandles[2], -vel)
simSetJointTargetVelocity(motorHandles[3], -vel)
simSetJointTargetVelocity(motorHandles[4], -vel)
end
function LeftAxis(vel)
vel = vel or 1
RobotMotion = "LeftAxis"
simSetJointTargetVelocity(motorHandles[1], vel)
simSetJointTargetVelocity(motorHandles[2], -vel)
simSetJointTargetVelocity(motorHandles[3], -vel)
simSetJointTargetVelocity(motorHandles[4], vel)
end    

function RightAxis(vel)
vel = vel or 1
RobotMotion = "RightAxis"
simSetJointTargetVelocity(motorHandles[1], -vel)
simSetJointTargetVelocity(motorHandles[2], vel)
simSetJointTargetVelocity(motorHandles[3], vel)
simSetJointTargetVelocity(motorHandles[4], -vel)
end    


function Joystick_GetCmd()
    if(simSerialCheck(Serial) >= 2) then
        JoyCmd = simSerialRead(Serial,2 ,false)
        local Jvel = 1
        --movement buttons
        if(JoyCmd == "00") then
            Stop()
        elseif(JoyCmd == "11") then
            Forward(Jvel)
        elseif(JoyCmd == "12") then
            RightAxis(Jvel)
        elseif(JoyCmd == "13") then
            Backward(Jvel)
        elseif(JoyCmd == "14") then
            LeftAxis(Jvel)
        end
        
        if(JoyCmd == "15") then --Get a new image and save it
            ImageRead = false
        end

        --increase/decrease velocity(Circle, X)
        if(JoyCmd == "02") then
            VelDx = VelDx + 10
        elseif(JoyCmd == "03") then
            if(VelDx > 20) then
                VelDx = VelDx - 10
            end
        end
    
    
        --Stop Simulation(Start)
        if(JoyCmd == "10") then
            simStopSimulation()
        end
    end
end

function GetEncoderData()
    for k=1,4 do
        Ang[k] = tonumber(simGetScriptSimulationParameter(SensorManager, IParameters[k])) or 0
        ABSAng[k] = tonumber(simGetScriptSimulationParameter(SensorManager, ABSParameters[k])) or 0
    end
end
function UpdateUi()
    --simSetUIButtonLabel(MyUi, 1, string.format('Ang[1] = %.3f',Ang[1]))
    --simSetUIButtonLabel(MyUi, 2, string.format('Ang[2] = %.3f',Ang[2]))
    --simSetUIButtonLabel(MyUi, 3, string.format('Ang[3] = %.3f',Ang[3]))
    --simSetUIButtonLabel(MyUi, 4, string.format('Ang[4] = %.3f',Ang[4]))simGetObjectOrientation(number objectHandle,number relativeToObjectHandle)
    
    simSetUIButtonLabel(MyUi, 5, string.format('MPos[1], Pos[1] = %.3f, %.3f',MyPosition[1], simMyPosition[1]))
    simSetUIButtonLabel(MyUi, 6, string.format('MPos[2], Pos[2] = %.3f, %.3f',MyPosition[2], simMyPosition[2]))
    simSetUIButtonLabel(MyUi, 7, string.format('MPos[3], Pos[3] = %.3f, %.3f',MyPosition[4], simMyPosition[4]))

    simSetUIButtonLabel(MyUi, 8, string.format('ErrorPos[1] = %.5f', math.abs(simMyPosition[1] - MyPosition[1])))
    simSetUIButtonLabel(MyUi, 9, string.format('ErrorPos[2] = %.5f', math.abs(simMyPosition[2] - MyPosition[2])))
    simSetUIButtonLabel(MyUi, 10, string.format('ErrorPos[3] = %.5f', math.abs(simMyPosition[4] - MyPosition[4])))
    simSetUIButtonLabel(MyUi, 11, "NumOfTurns = "..NumOfTurns)

    simSetUIButtonLabel(MyUi, 1, string.format('DistanceSonar[1] = %.3f',DistanceSonar[1]))
    simSetUIButtonLabel(MyUi, 2, string.format('DistanceSonar[2] = %.3f',DistanceSonar[2]))
    simSetUIButtonLabel(MyUi, 3, string.format('DistanceSonar[3] = %.3f',DistanceSonar[3]))
    simSetUIButtonLabel(MyUi, 4, string.format('DistanceSonar[4] = %.3f',DistanceSonar[4]))
end
function Position()

    simMyPosition[1] = simGetObjectPosition(RobotHandle, -1)[1]
    simMyPosition[2] = simGetObjectPosition(RobotHandle, -1)[2]
    simMyPosition[3] = GetAbsAngleFromJoint(simGetObjectOrientation(RobotHandle, -1)[3], 1)
    simMyPosition[4] = simMyPosition[3] * RAD2DEG

------------------------------------------------------------------------
    MyPosition[3] = StartPostion[3] + (R/(2*d))*(-(Ang[1] + Ang[3])/2.0 + (Ang[2] + Ang[4])/2.0)*DEG2RAD / 2 - 2 * math.pi * NumOfTurns
    
    if(MyPosition[3] >= 2*math.pi) then
        NumOfTurns = NumOfTurns + 1
    elseif(MyPosition[3] <= -2*math.pi) then
        NumOfTurns = NumOfTurns - 1
    end

    if(MyPosition[3] < 0) then
        MyPosition[3] = 2*math.pi + MyPosition[3]
    end

    MyPosition[4] = MyPosition[3]*RAD2DEG    
    

    if(RobotMotion ~= "LeftAxis" and RobotMotion ~= "RightAxis") then
        if(LRobotMotion == "LeftAxis" or LRobotMotion == "RightAxis") then
            for k=1,4 do
                Ang[k] = 0
                simSetScriptSimulationParameter(SensorManager, IParameters[k], Ang[k])
            end
            StartPostion[1] = MyPosition[1]
            StartPostion[2] = MyPosition[2]
            StartPostion[3] = MyPosition[3]
        end
        MyPosition[1] = StartPostion[1] + math.cos(simMyPosition[3])*(R/2)*((Ang[1] + Ang[3])/2.0 + (Ang[2] + Ang[4])/2.0)*DEG2RAD
        MyPosition[2] = StartPostion[2] + math.sin(simMyPosition[3])*(R/2)*((Ang[1] + Ang[3])/2.0 + (Ang[2] + Ang[4])/2.0)*DEG2RAD
    end
    LRobotMotion = RobotMotion
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


function GetSonarData()
    for k=1,4 do
        local result1, Dist = simReadProximitySensor(Sonico[k])--UltraSonico
        DistanceSonar[k] = Dist or -1
    end
end
function SendData()
    simSerialSend(Serial, string.format('%.3f',MyPosition[1])..string.format('%.3f',MyPosition[2])..string.format('%.3f',MyPosition[4])..string.format('%.3f',DistanceSonar[1])..string.format('%.3f',DistanceSonar[2])..string.format('%.3f',DistanceSonar[3])..string.format('%.3f',DistanceSonar[4]))
end
function SaveImageData(Camera, type)
    local ReadImage = simGetVisionSensorImage(Camera)

    if(ImageRead == false and ReadImage ~= nil) then
        ImageRead = true
        file_CAMERA = io.open(Image_filePath, "w")
        type = type or 0
        local img1 = {}
        local count = 0
        local MaxPixels = 256

        if(type == 0) then --Gray scale image
            for a = 1, MaxPixels do
                img1[a] = {}
                for b = 1, MaxPixels do
                    img1[a][b] = 0
                end        
            end

            local f = 1
            for i = 1,(MaxPixels) do
                for j = 1,(MaxPixels) do
                    for c = 1, 3 do
                    img1[MaxPixels + 1 - i][MaxPixels + 1 - j] = img1[MaxPixels + 1 - i][MaxPixels + 1 - j] + ReadImage[f]/3.0
                    f = f + 1
                    end
                end
            end


            for a = 1, MaxPixels do
                for b = 1, MaxPixels do
                    file_CAMERA:write(string.format('%03.1f', img1[a][MaxPixels + 1 - b]*255))

                    if(b == MaxPixels and a ~= MaxPixels) then
                        file_CAMERA:write('\n')
                    else
                        file_CAMERA:write(' ')
                    end
                end        
            end
        else --Colored image
            if(IsFirst) then
                for a = 1, MaxPixels do
                    img1[a] = {}
                    for b = 1, MaxPixels do
                        img1[a][b] = {}
                        for c = 1, 3 do
                            img1[a][b][c] = 0
                        end
                    end        
                end
                IsFirst = false
            end

            local f = 1
            for i = 1,(MaxPixels) do
                for j = 1,(MaxPixels) do
                    for c = 1, 3 do
                    img1[MaxPixels + 1 - i][MaxPixels + 1 - j][c] = ReadImage[f]
                    f = f + 1
                    end
                end
            end


            for a = 1, MaxPixels do
                for b = 1, MaxPixels do
                    file_CAMERA:write(string.format('%03.1f;%03.1f;%03.1f', img1[a][b][3]*255, img1[a][b][2]*255, img1[a][b][1]*255))

                    if(b == MaxPixels and a ~= MaxPixels) then
                        file_CAMERA:write('\n')
                    elseif(a ~= MaxPixels and b ~= MaxPixels) then
                        file_CAMERA:write(' ')
                    end
                end        
            end
        end
        file_CAMERA:close()
    end
end
    
function GetAbsAngleFromJoint(JointRawAngle, OutInDeg)
    OutInDeg = OutInDeg or 0
    local Angle = 0.0
    local ToDeg = 180.0 / math.pi
    if(JointRawAngle * ToDeg >= 0) then
        Angle = JointRawAngle * ToDeg
    else
        Angle = 360.0 + JointRawAngle * ToDeg
    end

    if(OutInDeg == 0) then
        return Angle
    else
        return Angle * 1.0 / ToDeg
    end
end
------------------------SLAM_FUNCTIONS_BEGIN------------------------------------
function ReadUltrasonicSensors()
--SonarSensors = {}
--SonarManager = simGetObjectHandle('SonarSensorManager')
    for k = 1, 11 do
        local detectedPoint = {}
        local theda = 0
        SensorsState[k], SensorsDistance[k] = simReadProximitySensor(SonarSensors[k])

        SensorsState[k] = SensorsState[k] or 0
        SensorsDistance[k] = SensorsDistance[k] or -1

        --local t = Sd[k] + SensorsDistance[k]

        theda = SensorFramnePosition[k]
        
        --D[k] = t

        --if(theda < 0) then
        --    theda = theda + 360
        --end

        theda = theda + simMyPosition[4]
        if(theda > 360) then
            SensorTheda[k] = theda - 360
        else
            SensorTheda[k] = theda
        end


        
        --simAuxiliaryConsolePrint(Console, string.format("Distance[%i] = %.3f / theda = %.3f\n", k, SensorsDistance[k], theda))
    end
    --simAuxiliaryConsolePrint(Console, "------------------------------------------------------------------------------\n")
end
function SendDataToSerial()
    local SerialString = string.format("%.3f", simMyPosition[1]).." "..string.format("%.3f", simMyPosition[2]).." "..string.format("%.3f", MyPosition[4])
    for k = 1, 11 do
        SerialString = SerialString.." "..string.format("%.3f", SensorsDistance[k]).." "..string.format("%.3f", SensorTheda[k])
    end
    simSerialSend(SLAM_Serial, SerialString)
end
------------------------SLAM_FUNCTIONS_END------------------------------------
--------------------------------------------------------------------------
end


if (sim_call_type==sim_childscriptcall_actuation) then

    -- Put your main ACTUATION code here
    Joystick_GetCmd()
    GetEncoderData()
    Position()
    UpdateUi()
    ReadUltrasonicSensors()

    if(simGetSimulationTime() - LastSerialTime > TimeSample) then
        LastSerialTime = simGetSimulationTime()
        SendDataToSerial()
    end

    --if(simGetSimulationTime() > 10) then
        --SaveImageData(RobotCamera, 0)
        --RightAxis(1)
        --if(simGetSimulationTime() - LTime > 1) then
            --LTime = simGetSimulationTime()
            --ImageRead = false
        --end
    --else
        --Forward(1)
    --end
    

    --Forward(0.1)
    --LeftAxis(1)
    --GetSonarData()
    

    --SendData()
end


if (sim_call_type==sim_childscriptcall_sensing) then

    -- Put your main SENSING code here
    --
end


if (sim_call_type==sim_childscriptcall_cleanup) then

    -- Put some restoration code here
    simSerialClose(Serial)
    simSerialClose(SLAM_Serial)
end