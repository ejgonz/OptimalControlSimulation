function J = getJerk(p,tf)
%GETJERK Returns jerk of 5th order polynomial p defined by over period [0,t]
%   p: vector of 6 polynomial coefficients
%   tf: final time of trajectory
    t = tf;
    J = 0.5*(36*p(4)^2*t + 144*p(4)*p(5)*t^2 + (240*p(4)*p(6)+192*p(5)^2)*t^3 ...
        + 720*p(5)*p(6)*t^4 + 720*(p(6)^2)*t^5);
end

