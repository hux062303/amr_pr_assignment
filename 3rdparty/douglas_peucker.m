function [list_pts,pnts]= douglas_peucker(point_list, tolerance)
%syntax [list_pts,pnts]= douglas_peucker(point_list, tolerance);
%this function approximate a polygon using "Douglas-Peucker Algorithm"
%inputs: 1. list of points on the polygon
%        2. tolerance
%the pseudo-code for this function is taken from 'http://www.cse.hut.fi/en/research/SVG/TRAKLA2/exercises/DouglasPeucker-212.html'
%the point_list has to be in the form [x y] coordinates. tolerance is the distance of points below which are ignored from the original point_list.
%the outputs are 1.list_pts which are the indices of the points from the point_list
%                2.pnts which will have the [x y] coordinates of the approximated curve
%principle: Check if points between start and end points fit into a tolerance area around the line segment between them. 
%           If yes, only start and end point are needed from the segment. 
%           If not, choose the farthest lying point as an end point and repeat check. 
%           When segment is handled move to the next segment which has current ending point
%           as start and most previous ending point as end point.
list_pts=[];    %initialise list
s={};           %initialise stack
indxfirst=1;    %index of first point
first=point_list(indxfirst,:);  %coordinates of first point
indxlast= size(point_list,1);   %index of last point
last=point_list(indxlast,:);    %coordinates of last point
s=push(indxlast,s);             
list_pts=[list_pts;indxfirst];
while(~isempty(s))              %terminate the iterations when stack is empty
        dmax=0;                 %initialise maximum distance as zero
    i=indxfirst+1;              %start iterations from the next point of first index
    while(i<indxlast)           %this loop computes shortest distance of a point from a line segment
        P=point_list(i,:);      %coordinates of point with index 'i'
        d=normal_dist(first,last,P);   % and compares it with the maximum distance value in previous iterations
        if (d>dmax)             %
            dmax=d;
            indxdist=i;
        end
        i=i+1;       
    end                         
    if(dmax<=tolerance)         %compare the maximum distance with tolerance and then the index is either pushed onto stack or list of output points
        list_pts=[list_pts;indxlast];
        [indxfirst,s]=pop(s);   %push and pop are user defined functions
        first=point_list(indxfirst,:);
        if(~isempty(s))
        indxlast=s{1};
                last=point_list(indxlast,:);
        end
    else
        indxlast=indxdist;
        last=point_list(indxlast,:);
        s=push(indxdist,s);
    end
end
pnts=point_list(list_pts,:);    %assign the coordinates of the points from the list of indices of point_list to output points.
return