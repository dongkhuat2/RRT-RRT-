clear all; close all;

x_I=100; y_I=100;           
x_G=700; y_G=700;       
Thr=50;                 
Delta= 30;              
T.v(1).x = x_I;         
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     
T.v(1).yPrev = y_I;
T.v(1).dist=0;          
T.v(1).indPrev = 0;     
findpath=false;
figure(1);
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
goal = [x_G,y_G];
start_goal_dist = 1000000;
path.pos(1).x = 700;
path.pos(1).y = 700;
dir=atan2(y_G-y_I,x_G-x_I);
tic
for iter = 1:1000
    x_rand=[];
%     x_rand(1) = xL*rand; 
%     x_rand(2) = yL*rand;
    x_rand(1) = xL*rand;
    a =rand-rand;
    x_rand(2)=tan(dir)*x_rand(1)+a*400;
 
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

    if ~collisionChecking(x_near,x_new,Imp) 
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
           
           if collisionChecking(x_neartmp,x_new,Imp)
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
        if (T.v(count).dist + norm(x_new - goal)) < start_goal_dist
            start_goal_dist = (T.v(count).dist + norm(x_new - goal));
            if length(path.pos) > 2
                for j = 2 : length(path.pos)
                    plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'w', 'Linewidth', 3);
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
                    plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
                
                end
            else
                disp('Error, no path found!');
            end
        end
        continue;
    end
    pause(0.01); 


end
toc
checkdist=0;
for j = 2:length(path.pos)
    checkdist=checkdist+norm([path.pos(j).x,path.pos(j).y]-[path.pos(j-1).x,path.pos(j-1).y])/100;
end
checkdist