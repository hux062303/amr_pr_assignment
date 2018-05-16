function line = lsqLine(edges)
    % line = LSQLINE(edges) extract a line described in (alpha,r)
    % parameters from a set of points
    
%     line = [0;0];
xs = sum(edges(1,:));
ys = sum(edges(2,:));
xxs = sum(edges(1,:).^2);
yys = sum(edges(2,:).^2);
xys = sum(edges(1,:).*edges(2,:));
n = size(edges,2);
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

end