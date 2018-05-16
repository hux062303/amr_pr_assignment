function H = Hest(p1, p2)

%% first, I do homography estimation without normalization
% A = zeros(3*size(p1,2),9);
% b = zeros(3*size(p1,2),1);
% for i = 1:1:size(p1,2)
%     k = (i-1)*3+1;
%     x1 = p1(1,i);
%     x2 = p2(1,i);
%     y1 = p1(2,i);
%     y2 = p2(2,i);
%     z1 = p1(3,i);
%     z2 = p2(3,i);
%     A(k:1:k+2,:) = [x1 y1 z1 0 0 0 0 0 0; ...
%                      0 0  0  x1 y1 z1 0 0 0; ...
%                      0 0 0 0 0 0 x1 y1 z1];
%     b(k:1:k+2,:) = [x2;y2;z2];
% end
% 
% Hv = inv(A'*A)*A'*b;
% H = [Hv(1) Hv(2) Hv(3);Hv(4) Hv(5) Hv(6);Hv(7) Hv(8) Hv(9)];

%% Then I do Homography estimation by normalization

[p1_normalized, T1] = normalize_pcl(p1);
[p2_normalized, T2] = normalize_pcl(p2);
% p1_normalized = p1;
% p2_normalized = p2;
% T1 = eye(3,3);
% T2 = eye(3,3);

A = zeros(3*size(p1_normalized,2),9);

for i = 1:1:size(p1_normalized,2)
    k = (i-1)*3+1;
    x1 = p1_normalized(1,i);
    x2 = p2_normalized(1,i);
    y1 = p1_normalized(2,i);
    y2 = p2_normalized(2,i);
    A(k,:)   = [0 -x1 x1*y2 0 -y1 y2*y1 0 -1 y2];
    A(k+1,:) = [x1 0 -x2*x1 y1 0 -y1*x2 1  0 -x2];
    A(k+2,:) = [-x1*y2 x2*x1 0 -y2*y1 y1*x2 0 -y2 x2 0];    
end
AA = A'*A;
[U S V] = svd(AA);
vmin = V(:,end);
Hn = [vmin(1) vmin(4) vmin(7);...
     vmin(2) vmin(5) vmin(8);...
     vmin(3) vmin(6) vmin(9)];

H = inv(T2)*Hn*T1; 
 
end