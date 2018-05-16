function [ so3 ] = so3_log( SO3 )
%UNTITLED8 此处显示有关此函数的摘要
%   此处显示详细说明
a = acos((trace(SO3)-1)*0.5);
% w = [SO3(3,2) - SO3(2,3); ...
%      SO3(1,3) - SO3(3,1); ...
%      SO3(2,1) - SO3(1,2)];
% 
% so3 = w/(2*(sin(a))+1e-12);
[U S V] = svd(SO3-eye(3,3));
so3 = V(:,end);
so3 = a .* so3;

end

