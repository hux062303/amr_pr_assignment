function [p_normalized, T1] = normalize_pcl(p)

%% p is 3xn
% Mean1=mean(p')';%% 3x1
% p(1,:)=p(1,:)-Mean1(1);
% p(2,:)=p(2,:)-Mean1(2);
% S1=mean(sqrt(diag(p'*p)))/sqrt(2);
% p(1:2,:)=p(1:2,:)/S1;
% T1=[eye(2)/S1,-Mean1(1:2)/S1;0 0 1];
% p_normalized = p;
Mean1 = mean(p');
p(1,:) = p(1,:) - Mean1(1);
p(2,:) = p(2,:) - Mean1(2);
% p(3,:) = p(3,:) - Mean1(1);
p12 = p(1:2,:);
S1 = sqrt(2)/mean(sqrt(diag(p12'*p12)));
p(1:2,:) = p(1:2,:)*S1;
T1=[S1 0 -Mean1(1)*S1; ... 
    0 S1 -Mean1(2)*S1; ...
    0 0 1];
p_normalized = p;

end