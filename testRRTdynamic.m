    clear all; close all;
    vrep=remApi('remoteApi');
    vrep.simxFinish(-1);
    %clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
    clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);
    vrep.simxStartSimulation(clientID,vrep.simx_opmode_oneshot);
    global path
    pose = [0,0,0];
    tireDiameter_m = 0.03;
    d = 0.0823;
    v = 0.1;
%     t=length(path.pos)
%     for i=1:length(path.pos)
%     pathx(length(path.pos)-i+1)=path.pos(i).x/100;
%     pathy(length(path.pos)-i+1)=path.pos(i).y/100;
%     end
%     goalPoints=[pathx' pathy']
    ii = 1;
    lastGoal = pose(1:2);
    currentGoal = [1,1];%%goalPoints(ii, :);
    atFinalGoal = 0;
    robPose(1,:)= pose;
    position=[1.000,1.000];
    wheel_radifront=0.035
    b=0.0823
    vTurn=0.03
    goalDetectedTol=2*pi/180;
    vref=0.1
    delta_t=0.05
    %const RRT
    x_I=100; y_I=100;           
    x_G=700; y_G=700;       
    Thr=100;                 
    Delta= 100;             
    T.v(1).x = x_I;         
    T.v(1).y = y_I; 
    T.v(1).xPrev = x_I;     
    T.v(1).yPrev = y_I;
    T.v(1).dist=0;          
    T.v(1).indPrev = 0;     
    findpath=false;
    
    f=figure(1);
    set(gcf,'Position',[100 100 200 200])

    ImpRgb=imread('newmap.png');
    Imp=rgb2gray(ImpRgb);
    imshow(Imp)
    hold on
    axis on;
    
    xL=size(Imp,1);
    yL=size(Imp,2);
    xticks(0:50:800);
    yticks(0:50:800);
    set(gca,'ydir','normal')
    xlabel('x (m)')
    ylabel('y (m)')
    hold on
    plot(x_I, y_I, 'ro', 'MarkerSize',10, 'MarkerFaceColor','r');
    plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');
    count=1;
    countr=1;
    goal = [x_G,y_G];
    start_goal_dist = 1000000;
    path.pos(1).x = 700;
    path.pos(1).y = 700;
    dir=atan2(y_G-y_I,x_G-x_I);
    iter=1;
    addobs=true;
    dem=1;
    dem1=1;
    obs_x=400;
    obs_y=750;
    cplot5=false
    
if (clientID>-1)
    % If connection successful
    disp('Connected')
    % Create handles for required V-Rep objects
    [returnCode,robot]=vrep.simxGetObjectHandle(clientID,'Robot',vrep.simx_opmode_blocking);
    [returnCode,dummy]=vrep.simxGetObjectHandle(clientID,'Dummy',vrep.simx_opmode_blocking);
    [returnCode,dyobs]=vrep.simxGetObjectHandle(clientID,'dynamic',vrep.simx_opmode_blocking)
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'motor_left',vrep.simx_opmode_blocking);
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'motor_right',vrep.simx_opmode_blocking);
    [returnCode,Orobot]=vrep.simxGetObjectHandle(clientID,'Robotpose',vrep.simx_opmode_blocking);
    [returnCode,Orierobot]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_streaming);
    [returnCode,Posirobot]=vrep.simxGetObjectPosition(clientID,Orobot,-1,vrep.simx_opmode_streaming);
 
    start=[x_I/100,y_I/100,0.052];
    [returnCode]=vrep.simxSetObjectPosition(clientID,robot,-1,start,vrep.simx_opmode_oneshot);
    
    while(~atFinalGoal)
            x_rand=[];
%     x_rand(1) = xL*rand; 
%     x_rand(2) = yL*rand;
    x_rand(1) = xL*rand;
    a =rand-rand
    x_rand(2)=tan(dir)*x_rand(1)+a*200;
 
    x_near=[];
    min_dist = 1000000;
    near_iter = 1;
    near_iter_tmp = 1;
    [~,N]=size(T.v);
    for j = 1:N
       x_near(1) = T.v(j).x;
       x_near(2) = T.v(j).y;
       dist = norm(x_rand - x_near);
       if min_dist > dist
           min_dist = dist;
           near_iter = j;
       end
    end
    x_near(1) = T.v(near_iter).x;
    x_near(2) = T.v(near_iter).y;

    x_new=[];
    near_to_rand = [x_rand(1)-x_near(1),x_rand(2)-x_near(2)];
    normlized = near_to_rand / norm(near_to_rand) * Delta;
    x_new = x_near + normlized;

    if ~collisionChecking(x_near,x_new,Imp,addobs,obs_x,obs_y) 
       continue;
    end
    
    nearptr = [];
    nearcount = 0;
    neardist =  norm(x_new - x_near) + T.v(near_iter_tmp).dist;  
    for j = 1:N
       if j == near_iter_tmp
           continue;
       end
       x_neartmp(1) = T.v(j).x;
       x_neartmp(2) = T.v(j).y;
       dist = norm(x_new - x_neartmp) + T.v(j).dist;
       norm_dist = norm(x_new - x_neartmp);
       if norm_dist < 120
           if collisionChecking(x_neartmp,x_new,Imp,addobs,obs_x,obs_y)
                nearcount = nearcount + 1;
                nearptr(nearcount,1) = j;
                if neardist > dist 
                    neardist = dist;
                    near_iter = j;
                end
           end
       end
    end
    
    x_near(1) = T.v(near_iter).x;
    x_near(2) = T.v(near_iter).y;
    count=count+1;

    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = x_near(1);     
	T.v(count).yPrev = x_near(2);
    T.v(count).dist= norm(x_new - x_near) + T.v(near_iter).dist;          
    T.v(count).indPrev = near_iter;   

    [M,~] = size(nearptr);
    for k = 1:M
        x_1(1) = T.v(nearptr(k,1)).x;
        x_1(2) = T.v(nearptr(k,1)).y;
        x1_prev(1) = T.v(nearptr(k,1)).xPrev;
        x1_prev(2) = T.v(nearptr(k,1)).yPrev;
        if T.v(nearptr(k,1)).dist >  (T.v(count).dist + norm(x_1-x_new))
            T.v(nearptr(k,1)).dist = T.v(count).dist + norm(x_1-x_new);
            T.v(nearptr(k,1)).xPrev = x_new(1);    
            T.v(nearptr(k,1)).yPrev = x_new(2);
            T.v(nearptr(k,1)).indPrev = count;
            c1(dem,k)=plot([x_1(1),x1_prev(1)],[x_1(2),x1_prev(2)],'-w');
            hold on;
            c2(dem,k)=plot([x_1(1),x_new(1)],[x_1(2),x_new(2)],'-g');
            hold on;
        end
    end
    %if (norm([x_new(1),x_new(2)]-[obs_x],[obs_y])>100)
    c3(iter)=plot([x_near(1),x_new(1)],[x_near(2),x_new(2)],'-r');
    hold on;
    c4(iter)=plot(x_new(1),x_new(2),'*r');
    hold on;
   % end
    
    if norm(x_new - goal) < Thr
        findpath=true;
        if (T.v(count).dist + norm(x_new - goal)) < start_goal_dist
            start_goal_dist = (T.v(count).dist + norm(x_new - goal));
            if length(path.pos) > 2
                for j = 2 : length(path.pos)
                    %c5(dem,j)=
                    cplot5=true;
                   % c5(j)=plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'w', 'Linewidth', 3);
%                        if(norm([path.pos(j).x,path.pos(j).y]-[obs_x,obs_y]))<100
%                        delete(c5(j));
%                    end
                end
            end
            path.pos = [];
            if iter < 2000
                path.pos(1).x = x_G; path.pos(1).y = y_G;
                path.pos(2).x = T.v(end).x; path.pos(2).y = T.v(end).y;
                pathIndex = T.v(end).indPrev; 
                j=0;
                while 1
                    path.pos(j+3).x = T.v(pathIndex).x;
                    path.pos(j+3).y = T.v(pathIndex).y;
                    pathIndex = T.v(pathIndex).indPrev;
                    if pathIndex == 1
                        break
                    end
                    j=j+1;
                end  
                path.pos(end+1).x = x_I; path.pos(end).y = y_I; 
                for j = 2:length(path.pos)
                   c6(j)= plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
                end

                %end
            else
                disp('Error, no path found!');
            end

        end
        continue;
    end
           for jj= 1:length(path.pos)
           if(norm([path.pos(jj).x,path.pos(jj).y]-[obs_x,obs_y]))<100
           for jjj=1:1:(jj+1)
               delete(c6(jjj));
               if cplot5 ==true
%                   delete(c5(jjj))
               end
           end
           end
                            end
        pause(0.01);
        %set goal point
        if findpath==true
        t=length(path.pos)
        for i=1:length(path.pos)
        pathx(length(path.pos)-i+1)=path.pos(i).x/100;
        pathy(length(path.pos)-i+1)=path.pos(i).y/100;
        end
        goalPoints=[pathx' pathy']
        end
        
        [returnCode,datapose]=vrep.simxGetStringSignal(clientID,'Robotpose',vrep.simx_opmode_buffer)
        if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
        position=vrep.simxUnpackFloats(datapose)
        end
       % [returnCode,position]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
        [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,Orobot,-1,vrep.simx_opmode_buffer)
        pose=[position(1,1),position(1,2),orientation(1,3)];
        
        %path following
        [w, gp] = purePursuit(pose, lastGoal, currentGoal, 0.2);
        vl = (2*v-d*w)/2;
        vr = (2*v+d*w)/2;
        if findpath==true
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
            end
        end
        end
        [returnCode,datapose]=vrep.simxGetStringSignal(clientID,'Robotpose',vrep.simx_opmode_buffer)
        if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
        position=vrep.simxUnpackFloats(datapose)
        end
        [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,Orobot,-1,vrep.simx_opmode_buffer)
        pose=[position(1,1),position(1,2),orientation(1,3)];
        plot (pose(1)*100,pose(2)*100,'-o','color','magenta');
        hold on;
        %if addobs==false
            if norm([pose(1)*100,pose(2)*100]-[obs_x,obs_y])<250
            addobs=true
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor,0,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, 0 ,vrep.simx_opmode_blocking);
            delete(c1);
            delete(c2);
            delete(c3);
            delete(c4);
            if cplot5==true
            delete(c5);
            end
            delete(c6);
            findpath=false;              
            clear goalPoints;
            break;
            %clear path
            end
    %end
            if ( obs_x<=550 && obs_y>=600)
            p = nsidedpoly(1000, 'Center', [obs_x obs_y], 'Radius', 100);
                obs_x=obs_x+1.3
                obs_y=obs_y-1.3
            [returnCode]=vrep.simxSetObjectPosition(clientID,dyobs,-1,[obs_x/100,obs_y/100,0.25],vrep.simx_opmode_oneshot);
            [returnCode]=vrep.simxSetObjectOrientation(clientID,dyobs,-1,[0 0 0],vrep.simx_opmode_oneshot);
            c(dem+1,dem1+1)=plot(p, 'FaceColor', 'black')
            delete(c(dem,dem1));
            dem=dem+1
            dem1=dem1+1 
            end
        iter=iter+1;

    end
    
    %% REPLAN
    Replan.r(1).x = pose(1)*100;         
    Replan.r(1).y = pose(2)*100; 
    Replan.r(1).xPrev = pose(1)*100;     
    Replan.r(1).yPrev = pose(2)*100;
    Replan.r(1).dist=0;          
    Replan.r(1).indPrev = 0; 
    pose_replan_x=pose(1);
    pose_replan_y=pose(2)
    Thr=20;
    Delta= 50; 
    iter=1; 
    ii=1
    currentGoal=[pose_replan_x,pose_replan_y];
     while(~atFinalGoal)
            x_rand=[];
    x_rand(1) = (xL-pose_replan_x)*rand+pose_replan_x*100-200; 
    x_rand(2) = (yL-pose_replan_y)*rand+pose_replan_y*100-100;
%     x_rand(1) = xL*rand;
%     a =rand-rand
%     x_rand(2)=tan(dir)*x_rand(1)+a*200;
    
    x_near=[];
    min_dist = 1000000;
    near_iter = 1;
    near_iter_tmp = 1;
    [~,N]=size(Replan.r);
    for j = 1:N
       x_near(1) = Replan.r(j).x;
       x_near(2) = Replan.r(j).y;
       dist = norm(x_rand - x_near);
       if min_dist > dist
           min_dist = dist;
           near_iter = j;
       end
    end
    x_near(1) = Replan.r(near_iter).x;
    x_near(2) = Replan.r(near_iter).y;

    x_new=[];
    near_to_rand = [x_rand(1)-x_near(1),x_rand(2)-x_near(2)];
    normlized = near_to_rand / norm(near_to_rand) * Delta;
    x_new = x_near + normlized;

    if ~collisionChecking(x_near,x_new,Imp,addobs,obs_x,obs_y) 
       continue;
    end
    
    nearptr = [];
    nearcount = 0;
    neardist =  norm(x_new - x_near) + Replan.r(near_iter_tmp).dist;  
    for j = 1:N
       if j == near_iter_tmp
           continue;
       end
       x_neartmp(1) = Replan.r(j).x;
       x_neartmp(2) = Replan.r(j).y;
       dist = norm(x_new - x_neartmp) + Replan.r(j).dist;
       norm_dist = norm(x_new - x_neartmp);
       if norm_dist < 120
           if collisionChecking(x_neartmp,x_new,Imp,addobs,obs_x,obs_y)
                nearcount = nearcount + 1;
                nearptr(nearcount,1) = j;
                if neardist > dist 
                    neardist = dist;
                    near_iter = j;
                end
           end
       end
    end
    
    x_near(1) = Replan.r(near_iter).x;
    x_near(2) = Replan.r(near_iter).y;
    countr=countr+1;

    Replan.r(countr).x = x_new(1);
    Replan.r(countr).y = x_new(2); 
    Replan.r(countr).xPrev = x_near(1);     
	Replan.r(countr).yPrev = x_near(2);
    Replan.r(countr).dist= norm(x_new - x_near) + Replan.r(near_iter).dist;          
    Replan.r(countr).indPrev = near_iter;   

    [M,~] = size(nearptr);
    for k = 1:M
        x_1(1) = Replan.r(nearptr(k,1)).x;
        x_1(2) = Replan.r(nearptr(k,1)).y;
        x1_prev(1) = Replan.r(nearptr(k,1)).xPrev;
        x1_prev(2) = Replan.r(nearptr(k,1)).yPrev;
        if Replan.r(nearptr(k,1)).dist >  (Replan.r(countr).dist + norm(x_1-x_new))
            Replan.r(nearptr(k,1)).dist = Replan.r(countr).dist + norm(x_1-x_new);
            Replan.r(nearptr(k,1)).xPrev = x_new(1);    
            Replan.r(nearptr(k,1)).yPrev = x_new(2);
            Replan.r(nearptr(k,1)).indPrev = countr;
            plot([x_1(1),x1_prev(1)],[x_1(2),x1_prev(2)],'-w');
            hold on;
            plot([x_1(1),x_new(1)],[x_1(2),x_new(2)],'-g');
            hold on;
        end
    end
    
    plot([x_near(1),x_new(1)],[x_near(2),x_new(2)],'-r');
    hold on;
    plot(x_new(1),x_new(2),'*r');
    hold on;
    if norm(x_new - goal) < Thr
        findpath=true;
        if (Replan.r(countr).dist + norm(x_new - goal)) < start_goal_dist
            start_goal_dist = (Replan.r(countr).dist + norm(x_new - goal));
            if length(path.pos) > 2
                for j = 2 : length(path.pos)
                    plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'w', 'Linewidth', 3);
                end
            end
            path.pos = [];
            if iter < 2000
                path.pos(1).x = x_G; path.pos(1).y = y_G;
                path.pos(2).x = Replan.r(end).x; path.pos(2).y = Replan.r(end).y;
                pathIndex = Replan.r(end).indPrev; 
                j=0;
                while 1
                    path.pos(j+3).x = Replan.r(pathIndex).x;
                    path.pos(j+3).y = Replan.r(pathIndex).y;
                    pathIndex = Replan.r(pathIndex).indPrev;
                    if pathIndex == 1
                        break
                    end
                    j=j+1;
                end  
                path.pos(end+1).x = pose_replan_x*100; path.pos(end).y = pose_replan_y*100; 
                for j = 2:length(path.pos)
                    plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
                end
            else
                disp('Error, no path found!');
            end
        end
        continue;
    end
    pause(0.01); 
    if findpath==true
            t=length(path.pos)
    for i=1:length(path.pos)
    pathx(length(path.pos)-i+1)=path.pos(i).x/100;
    pathy(length(path.pos)-i+1)=path.pos(i).y/100;
    end
    goalPoints=[pathx' pathy']
    end
    
        [returnCode,datapose]=vrep.simxGetStringSignal(clientID,'Robotpose',vrep.simx_opmode_buffer)
        if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
        position=vrep.simxUnpackFloats(datapose)
        end
       % [returnCode,position]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
        [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,Orobot,-1,vrep.simx_opmode_buffer)
        pose=[position(1,1),position(1,2),orientation(1,3)];
        
        %path following
        [w, gp] = purePursuit(pose, lastGoal, currentGoal, 0.3);
        vl = (2*v-d*w)/2;
        vr = (2*v+d*w)/2;
        if findpath==true
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
                findpath =false
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, 0,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, 0,vrep.simx_opmode_blocking);
            end
        end
        end
        [returnCode,datapose]=vrep.simxGetStringSignal(clientID,'Robotpose',vrep.simx_opmode_buffer)
        if (returnCode==vrep.simx_return_ok) % After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
        position=vrep.simxUnpackFloats(datapose)
        end
        [returnCode,orientation]=vrep.simxGetObjectOrientation(clientID,Orobot,-1,vrep.simx_opmode_buffer);
        [returnCode,position]=vrep.simxGetObjectPosition(clientID,Orobot,-1,vrep.simx_opmode_buffer)
        pose=[position(1,1),position(1,2),orientation(1,3)];
        plot (pose(1)*100,pose(2)*100,'-o','color','magenta');
        hold on;
        iter=iter+1
        %dem=dem+1
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
