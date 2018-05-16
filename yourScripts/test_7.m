function test_7
close all;
clc;
addpath('/Users/xiaohu/Downloads/kalmanexercise/extras');
%% laser pose
x = 0;
y = 0;
theta = 0;
%% obstable lines
lines = [[3,-4,3,1]'];
%% resolution, fov and maximum range
resol = 0.36;
fov = 180;
maxdistance = 4;

%% generate scane
scan = laserscan(x,y,theta,lines,maxdistance,resol);
figure(1);
polarplot(scan(1,:),scan(2,:),'r.','MarkerSize',2);
title('Polar');
%% polar to carth
pos_carth = polar2carth(scan);
figure(2)
plot(pos_carth(1,:),pos_carth(2,:),'.','MarkerSize',2);
title('Carth local');
%% local to global
pos_laser = [x;y;theta];
pglobal = transform(pos_carth, pos_laser);
figure(3)
plot(pglobal(1,:),pglobal(2,:),'.','MarkerSize',2);
title('Carth global');
%% complex example
complex_lines = [[0,1,2.5,1]' ...
                 [2.5,1,2.5,5]' ...
                 [0,-1,3.5,-1]' ...
                 [3.5,-1,3.5,5]'];

x = 0; y = 0; theta = 0;
pos_laser1 = [x;y;theta];
scan1 = laserscan(x,y,theta,complex_lines,maxdistance,resol);
x = 1; y = 0; theta = 0;
pos_laser2 = [x;y;theta];
scan2 = laserscan(x,y,theta,complex_lines,maxdistance,resol);
x = 1.5; y = 0; theta = pi/6;
pos_laser3 = [x;y;theta];
scan3 = laserscan(x,y,theta,complex_lines,maxdistance,resol);

pos_carth1 = polar2carth(scan1);
pos_carth2 = polar2carth(scan2);
pos_carth3 = polar2carth(scan3);

pglobal1 = transform(pos_carth1, pos_laser1);
pglobal2 = transform(pos_carth2, pos_laser2);
pglobal3 = transform(pos_carth3, pos_laser3);

figure(4);
hold on;
plot(pglobal1(1,:),pglobal1(2,:),'r.','MarkerSize',2);
plot(pglobal2(1,:),pglobal2(2,:),'go','MarkerSize',2);
plot(pglobal3(1,:),pglobal3(2,:),'y*','MarkerSize',2);

%% lsq line fitting

%% problem 5
% x = 0; y = 0; theta = 0;
% line_fitting = [-1 -2 1 -4]';
% scan_fitting = laserscan(x,y,theta,line_fitting,8,resol);
% pos_carth_fitting = polar2carth(scan_fitting);
% figure(5);
% plot(pos_carth_fitting(1,:),pos_carth_fitting(2,:),'.');
% points = pos_carth_fitting(:,(scan_fitting(2,:)<8));
% hold on;
% plot(points(1,:),points(2,:),'r.');
% line = lsqline(points);

%% automatically extract lines
figure(5);
[n,id] = lineprune(scan3, pos_carth3, maxdistance);
plot(pglobal3(1,:),pglobal3(2,:),'.','MarkerSize',2);

for i = 1:1:n
    pts = pos_carth3(:,id == i);
    hold on;
    
    %% lsq fit in local frame
    line_local = lsqline(pts);
    
    %% lsq fit in global frame
    pts_global = pglobal3(:,id == i);
    plot(pts_global(1,:),pts_global(2,:),'go');

    line_global = lsqline(pts_global);

    %% transform from local to global and compare
    line_global_t = transformline(line_local, pos_laser3);
end


return

function line_w = transformline(line_l, systempose_w)
t = line_l(1,:);
r = line_l(2,:);

%% norm
line_norm = [cos(t);sin(t)];
point_on_line = [r.*cos(t);r.*sin(t)];

%% transform
tw = systempose_w(3);
line_norm_w = [cos(tw) -sin(tw);sin(tw) cos(tw)]*line_norm;
point_on_line_w = [cos(tw) -sin(tw);sin(tw) cos(tw)]*point_on_line;
point_on_line_w(1,:) = point_on_line_w(1,:) + systempose_w(1);
point_on_line_w(2,:) = point_on_line_w(2,:) + systempose_w(2);

%% compute r
r_global = point_on_line_w(1,:).*line_norm_w(1,:)+point_on_line_w(2,:).*line_norm_w(2,:);
id = r_global < 0;
r_global(id) = -r_global(id);
line_norm_w(:,id) = line_norm_w(:,id)*-1;

%% compute theta
theta_w = atan2(line_norm_w(2,:),line_norm_w(1,:));

line_w = [theta_w;r_global];

return

function [n,id] = lineprune(scan, carth, max_r)

idvalid = scan(2,:)<max_r;
n = 1;
i=1;
id = zeros(size(carth,2),1);
min_line_support_num = 5;
while i<size(carth,2)
    if idvalid(i)~=0
%         id(i) = n;
%         id(i+1) = n;
        %% construct a ray
        v = carth(:,i+1) - carth(:,i);
        v = v./norm(v);
        j = i+2;
        while j < size(carth,2)
            
            vtest = carth(:,j) - carth(:,j-2);
            vtest = vtest./norm(vtest);
            
            %% cross-angle
            online = abs(dot(v,vtest));
            if abs(online - 1) < 0.01
                j = j+1;
            else
                break;
            end
            
            v = carth(:,j-1) - carth(:,j-2);
            v = v./norm(v);
            
        end
        num = j - i;
        if (num >= min_line_support_num)
            id(i:1:j-1) = id(i:1:j-1) + n;
            n = n + 1;
        end
        i = j;
    end
end

return

function line = lsqline(points)
xs = sum(points(1,:));
ys = sum(points(2,:));
xxs = sum(points(1,:).^2);
yys = sum(points(2,:).^2);
xys = sum(points(1,:).*points(2,:));
n = size(points,2);
alpha = 0.5*atan2(2*xs*ys-2*n*xys,xs*xs-ys*ys-n*xxs+n*yys);
xmean = xs/n;
ymean = ys/n;
r = xmean*cos(alpha)+ymean*sin(alpha);
if r < 0
    r = -r;
    if alpha < 0
        alpha = alpha + pi;
    else
        alpha = alpha - pi;
    end
end
line = [alpha;r];
return

function pos_carth = polar2carth(pol)
xl = cos(pol(1,:)).*pol(2,:);
yl = sin(pol(1,:)).*pol(2,:);
pos_carth = [xl;yl;pol(1,:)];
return

function pow_w = transform(pos_l, systempose_w)

pow_w = zeros(3,size(pos_l,2));

pow_w(1,:) = cos(systempose_w(3)).*pos_l(1,:)-sin(systempose_w(3)).*pos_l(2,:) + systempose_w(1);
pow_w(2,:) = sin(systempose_w(3)).*pos_l(1,:)+cos(systempose_w(3)).*pos_l(2,:) + systempose_w(2);
pow_w(3,:) = pos_l(3,:)+systempose_w(3);

return