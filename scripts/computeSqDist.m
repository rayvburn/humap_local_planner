clear all;
close all;
clc;

p0 = [0 0];
p1 = [2 1];
p2 = [3 4];
p3 = [4 1];

d03 = sqDist(p3, p0)
d03_segments = sqDist(p3, p2) + sqDist(p2, p1) + sqDist(p1, p0)

d03_segments_eucl = euDist(d03_segments)
d03_segments_eucl_ver = euDist(sqDist(p3, p2)) + euDist(sqDist(p2, p1)) + euDist(sqDist(p1, p0))

function dist_sq = sqDist(p1, p2)
  distx = p2(1) - p1(1);
  disty = p2(2) - p1(2);
  dist_sq = distx * distx + disty * disty;
end

function dist = euDist(sq_dist)
    dist = sqrt(sq_dist);
end
