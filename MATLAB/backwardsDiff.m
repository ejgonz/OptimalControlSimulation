function [v,a] = backwardsDiff(x,dt)
%backwardsDiff Returns backwards difference approximation for velocity and
%acceleration given position data x and timestep dt.

v = diff(x)./dt; 
a = diff(v)./dt;
end

