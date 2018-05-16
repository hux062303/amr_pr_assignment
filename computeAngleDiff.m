function angleDiff = computeAngleDiff(a, b)

ax = sin(a);ay = cos(a);
bx = sin(b);by = cos(b);

sz = (bx*ay-by*ax);
cz = ax*bx+ay*by;

angleDiff = atan2(sz,cz);

end