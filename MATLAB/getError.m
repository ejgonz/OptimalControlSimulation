function e = getError(x0,xd0,xdd0,xf,tf,xp,dt)
% GETERROR Get square error of polynomial formed from initial conditions
% (x0,xd0,xdd0) and final conditions (xt,tf) as compared to the previous
% points xp, with dt sampling time. Also returns polynomial p.

k = length(xp); % Number of past points

% Solve for polynomial given BC (0,x0) --> (tf,xf)
p = solveMinJerk(x0,xd0,xdd0,xf,tf);
p = flipud(p);

% Get time vector
tp = -dt*(k-1):dt:0; % time vector for previous datapoints [-(k-1)dt, 0]

% Evaluate polynomial estimate using time vector
x = polyval(p,tp)';

% Compute error and return
e = (x - xp)'*(x - xp);

% %% Plot Intermediate steps
% t = -dt*(k-1):dt:tf;
% plot(tp,xp,'o'); hold on;
% plot(t,polyval(p,t),'--');
% title('X Trajectory');
% xlabel('time s');
% ylabel('pos cm');
% legend('data points','fit trajectory')
end

