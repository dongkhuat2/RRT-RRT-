
clear all; close all;
x_I=100; y_I=100;           %init point
x_G=700; y_G=700;       %goal
Thr=50;                 %distance check to goal
Delta= 30;              % const to get xnew

T.v(1).x = x_I;         
T.v(1).y = y_I; 
T.v(1).xPrev = x_I;     
T.v(1).yPrev = y_I;
T.v(1).dist=0;          
T.v(1).indPrev = 0;     %

figure(1);
ImpRgb=imread('newmap.png');
Imp=rgb2gray(ImpRgb);
imshow(Imp)
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
plot(x_G, y_G, 'go', 'MarkerSize',10, 'MarkerFaceColor','g');%
count=1;
goal = [x_G,y_G];
dir=atan2(y_G-y_I,x_G-x_I);
tic
pl=true

for iter = 1:500
    x_rand=[];
    x_rand(1) = xL*rand; 
    x_rand(2) = yL*rand;
%     x_rand(1) = xL*rand;
%     a =rand-rand;
%     x_rand(2)=tan(dir)*x_rand(1)+a*400;
    x_near=[];
    min_dist = 1000000;
    near_iter = 1;
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
    count=count+1;

    T.v(count).x = x_new(1);
    T.v(count).y = x_new(2); 
    T.v(count).xPrev = x_near(1);     
	T.v(count).yPrev = x_near(2);
    T.v(count).dist= norm(x_new - x_near);          
    T.v(count).indPrev = near_iter;   
    plot([x_near(1),x_new(1)],[x_near(2),x_new(2)],'-r');
    hold on;

    if (norm(x_new - goal) < Thr) && pl== true
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
        pl=false;
        plot([path.pos(j).x; path.pos(j-1).x;], [path.pos(j).y; path.pos(j-1).y], 'b', 'Linewidth', 3);
    end
else
    disp('Error, no path found!');
        end
    end
    if iter>=1000
        break;
    end
    pause(0.01);
end

toc
checkdist=0;
for j = 2:length(path.pos)
    checkdist=checkdist+norm([path.pos(j).x,path.pos(j).y]-[path.pos(j-1).x,path.pos(j-1).y])/100;
end
checkdist
