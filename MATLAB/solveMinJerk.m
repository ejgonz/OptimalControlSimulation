function [a] = solveMinJerk(x0,xd0,xdd0,xf,tf)
%SOLVEMINJERK Finds 1D minimum jerk trajectory given initial and final
%conditions
%   Assumptions: t0 = 0; xfdot = xfddot = 0;
%   Refer to Fast & Hogan (1985) Appendix A

% System of 6 linear equations
A = [1 0 0 0 0 0; ...
     0 1 0 0 0 0;...
     0 0 2 0 0 0;...
     1 tf tf^2 tf^3 tf^4 tf^5;...
     0 1 2*tf 3*tf^2 4*tf^3 5*tf^4; ...
     0 0 2 6*tf 12*tf^2 20*tf^3];
 
b = [x0; xd0; xdd0; xf; 0; 0];
a = A\b;
end

