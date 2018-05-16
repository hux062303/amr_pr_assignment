function [ projectedLine, lineCov ] = projectToLaser( worldLine,poseIn, covIn)
%[projectedLine, lineCov] = PROJECTTOLASER(worldLine,poseIn,covIn) 
%Project a word line to the laser scanner frame given the
%world line, the robot pose and robot pose covariance. Note that the laser
%scanner pose in the robot frame is read globally
%   worldLine: The line in world coordinates
%   poseIn: The robot pose
%   covIn: The robot pose covariance
%
%   projectedLine: The line parameters in the laser scanner frame
%   lineCov: The covariance of the line parameters

%% Constants
global lsrRelPose % The laser scanner pose in the robot frame is read globally
global varAlpha varR

%% Calculation
% projectedLine = [0,0];
%% from world to robot
laseringlobal = [cos(poseIn(3)) -sin(poseIn(3));sin(poseIn(3)) cos(poseIn(3))] * [lsrRelPose(1);lsrRelPose(2)] ...
    + [poseIn(1);poseIn(2)];

ar = worldLine(1,:) - poseIn(3) - lsrRelPose(3);
rr = worldLine(2,:) - (laseringlobal(1)).*cos(worldLine(1,:)) - laseringlobal(2).*sin(worldLine(1,:));
projectedLine(1) = ar;
projectedLine(2) = rr;


% lineCov = zeros(2,2);
a = worldLine(1,:);
C = [0 0 -1;...
    -cos(a) -sin(a) ...
    -cos(a).*(-sin(poseIn(3)).*lsrRelPose(1)-cos(poseIn(3)).*lsrRelPose(2))-sin(a).*(cos(poseIn(3)).*lsrRelPose(1)-sin(poseIn(3)).*lsrRelPose(2))];
R = [varAlpha 0;0 varR];
lineCov = C*covIn*C'+R;
end
