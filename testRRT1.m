    vrep=remApi('remoteApi');
    vrep.simxFinish(-1);
%     clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    pose = [1,1,0];
    tireDiameter_m = 0.03;
    d = 0.0823;
    v = 0.1;
    t=length(path.pos)
    for i=1:length(path.pos)
    pathx(length(path.pos)-i+1)=path.pos(i).x/100;
    pathy(length(path.pos)-i+1)=path.pos(i).y/100;
    end
    goalPoints=[pathx' pathy']
    ii = 1;
    lastGoal = pose(1:2);
    currentGoal = goalPoints(ii, :);
    atFinalGoal = 0;
    robPose(1,:)= pose;
    position=[1.000,1.000];
    state =1;
    wheel_radifront=0.035
    b=0.0823
    vTurn=0.03
    goalDetectedTol=1*pi/180;
    vref=0.1
    delta_t=0.05
    dem=0
if (clientID>-1)
    % If connection successful
    disp('Connected')
    % Create handles for required V-Rep objects
    [returnCode,robot]=vrep.simxGetObjectHandle(clientID,'Robot',vrep.simx_opmode_blocking)
    [returnCode,dummy]=vrep.simxGetObjectHandle(clientID,'Dummy',vrep.simx_opmode_blocking);
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'motor_left',vrep.simx_opmode_blocking);
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'motor_right',vrep.simx_opmode_blocking);
    [returnCode,Orobot]=vrep.simxGetObjectHandle(clientID,'Robotpose',vrep.simx_opmode_blocking)
    [returnCode,Orierobot]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_streaming)
    [returnCode,Posirobot]=vrep.simxGetObjectPosition(clientID,Orobot,-1,vrep.simx_opmode_streaming)
    start=[1,1,0.052];
    [returnCode]=vrep.simxSetObjectPosition(clientID,robot,-1,start,vrep.simx_opmode_oneshot);
    pause(0.1);
    while(~atFinalGoal)
        [returnCode,datapose]=vrep.simxGetStringSignal(clientID,'Robotpose1',vrep.simx_opmode_buffer)
        if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
        position=vrep.simxUnpackFloats(datapose)
        end
       % [returnCode,position]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
        [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,Orobot,-1,vrep.simx_opmode_buffer)
        pose=[position(1,1),position(1,2),orientation(1,3)];
        
        plot (pose(1)*100,pose(2)*100,'color','magenta');
        hold on;

       % path following
        [w, gp] = purePursuit(pose, lastGoal, currentGoal, 0.2);
        vl = (2*v-d*w)/2;
        vr = (2*v+d*w)/2;
        
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, vl/tireDiameter_m,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, vr/tireDiameter_m ,vrep.simx_opmode_blocking);
        % Update goal points if target goal reached
        atGoal=atGoalPoint(pose, currentGoal) 
        if atGoal ==true
            if(ii < length(goalPoints))  
                % increment goal point
                lastGoal = currentGoal;
                ii = ii + 1;
                currentGoal = goalPoints(ii, :);         
            else
                % No more goal points in list; at final goal
                atFinalGoal = 1;
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, 0,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, 0 ,vrep.simx_opmode_blocking);
            end
        end
        [returnCode,datapose]=vrep.simxGetStringSignal(clientID,'Robotpose',vrep.simx_opmode_buffer)
        if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
        position=vrep.simxUnpackFloats(datapose)
        end
        [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,Orobot,-1,vrep.simx_opmode_buffer)
        pose=[position(1,1),position(1,2),orientation(1,3)];
        %state=1;
        dem=dem+1
    end
     vrep.simxFinish(-1);
     %[returnCode]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)

else
    % Connection Failed
    disp('Failed connecting to remote API server')
end

function [atGoal] = atGoalPoint(robotPose, goalPoint)
    goalRadius_m = 0.2;
    dist = sqrt((robotPose(1) - goalPoint(1))^2 + (robotPose(2) - goalPoint(2))^2);
    if abs(dist) < goalRadius_m
         atGoal = true;
    else atGoal=false;
    end
end