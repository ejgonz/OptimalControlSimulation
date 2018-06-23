function [px, py, x, y, t, J, e] = getPolyEst2D(xp,xt,dt)
%getPolyEst1D Returns polynomial estimate from current state to target.
%   Inputs
%   xp: Set of previous (x,y) datapoints (kx2)
%   xt: Target x,y position [2 x 1]
%   dt: Timestep
%
%   Outputs
%   p = 5th order polynomial coefficients of estimate
%   x = x values of polynomial estimate
%   t = time values of polynomial estimate (-(k-1)dt,tf)
%   J = Jerk of polynomial estimate

%% Estimate Initial Conditions
[vpx,apx] = backwardsDiff(xp(:,1),dt);
[vpy,apy] = backwardsDiff(xp(:,2),dt);
x0   = xp(end,1);  % Initial Position
xd0  = vpx(end);   % Initial Velocity
xdd0 = apx(end);   % Initial Acceleration

y0   = xp(end,2);  % Initial Position
yd0  = vpy(end);   % Initial Velocity
ydd0 = apy(end);   % Initial Acceleration

%% Find optimal polynomial
tol = dt;
options = optimset('Tolx',tol);
[tf_est_x, ex] = fminbnd(@(tf) getError(x0,xd0,xdd0,xt(1),tf,xp(:,1),dt), 0.001, 0.5+dt, options);
[tf_est_y, ey] = fminbnd(@(tf) getError(y0,yd0,ydd0,xt(2),tf,xp(:,2),dt), 0.001, 0.5+dt, options);
e = sqrt(ex^2 + ey^2);

% fprintf('tf x estimate: %d \n',tf_est_x);
% fprintf('estimate x error: %d \n \n',ex);
% fprintf('tf y estimate: %d \n',tf_est_y);
% fprintf('estimate y error: %d \n \n',ey);

% Solve for polynomial given BC (0,x0) --> (tf,xf)
px = solveMinJerk(x0,xd0,xdd0,xt(1),tf_est_x);
px = flipud(px);
py = solveMinJerk(y0,yd0,ydd0,xt(2),tf_est_y);
py = flipud(py);

% Evaluate polynomial
k = length(xp);      % number of previous points
tf_est = max(tf_est_x,tf_est_y);
t = 0:dt:tf_est; % [-(k-1)dt, tf]
x = polyval(px,t);
y = polyval(py,t);

% Cut off vector at first time target is reached
diffx = x-xt(1);
diffy = y-xt(2);

if (diffx(1) > 0)
    i = find(diffx <= 0,1); % Find first time x = xt
else
    i = find(diffx >= 0,1);
end
if (diffy(1) > 0)
    j = find(diffy <= 0,1); % Find first time y = yt
else
    j = find(diffy >= 0,1);
end
i = max(i,j);

if (t(i) < tf_est)
    tf_est = t(i);
    t = -dt*(k-1):dt:tf_est; % [-(k-1)dt, tf]
    x = polyval(px,t); 
    y = polyval(py,t);
end

%% Compute Jerk over interval (0,tf)
J = getJerk(px,tf_est) + getJerk(py,tf_est);
end

