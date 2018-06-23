close all;
clear all;
yalmip('clear');

load('Dataset1.mat');
Xtargets = Xtargets([1 3],:);   % Grab only x and y of target positions

% Declare global variables (for plotting)
global xhL xhR reachColors Xtargets xr_actual x1min x2min x1max x2max u_actual

xr0L = [-35;-10];     % Initial condition of left robot
xr0R = [-20;0];     % Initial condition of right robot
xr0 = [xr0L; xr0R];

xr_actual = xr0;    % Robot trajectory
xhL = [];
u_actual = [];
T = 6;              % Horizon

% Interpolate data points (simulate sample time of 0.01)
numInterp = 2;
handPosInterpX = interp1q((1:length(handPos))',handPos(:,1),(1:1/numInterp:length(handPos))');
handPosInterpY = interp1q((1:length(handPos))',handPos(:,3),(1:1/numInterp:length(handPos))');
handPosInterp = [handPosInterpX handPosInterpY];

% Cost
pathCost = 0;

% Reaching variables
reachVelThresh = 20;
reachColors = [];

avgExecutionTime = 0;
avgSetupTime = 0;
numSamples = 100;
leftHandStartSample = 471;
rightHandStartSample = 494;

%% YALMIP FORMULATION
% Model data
nx = 4; % Number of states
nu = 4; % Number of inputs

% MPC data
r = 0.00;
dt = 0.05/numInterp;

% State constraints
x1max = 5; x1min = -55;
x2max = 7.5; x2min = -20;
xmax = [x1max; x2max; x1max; x2max];
xmin = [x1min; x2min; x1min; x2min];

% Control constraints
maxspeed = 60;
umax2 = [maxspeed^2; maxspeed^2];
C = [1 0 0 0; 0 0 1 0];
D = [0 1 0 0; 0 0 0 1];

E = [1 0 -1 0; 0 1 0 -1];

% Optimization Variables
u = sdpvar(repmat(nu,1,T),repmat(1,1,T));
x = sdpvar(repmat(nx,1,T+1),repmat(1,1,T+1));
WXT = sdpvar(repmat(1,1,T),repmat(nx,1,T));

constraints = [];
objective = 0;
for k = 1:T
 objective = objective + x{k}'*x{k} - 2*WXT{k}*x{k} + r*u{k}'*u{k};
 constraints = [constraints, x{k+1} == x{k} + dt*u{k}];
 constraints = [constraints, (C*u{k}).^2 + (D*u{k}).^2 <= umax2];
 
% State Constraints
 constraints = [constraints, x1min <= x{k}(1) <= x1max,...
                             x2min <= x{k}(2) <= x2max,...
                             x1min <= x{k}(3) <= x1max,...
                             x2min <= x{k}(4) <= x2max];

 % Collision Avoidance
 SafetyRegionRight = [x{k}(1) - x{k}(3) >= 10];
 SafetyRegionLeft  = [x{k}(3) - x{k}(1) >= 10];
 SafetyRegionAbove = [x{k}(2) - x{k}(4) >= 10];
 SafetyRegionBelow = [x{k}(4) - x{k}(2) >= 10];
 
 constraints = [constraints, SafetyRegionRight|SafetyRegionLeft|SafetyRegionAbove|SafetyRegionBelow];
end

parameters_in = {x{1},WXT{:}};
solutions_out = {[u{:}], [x{:}]};

controller = optimizer(constraints, objective,sdpsettings('solver','gurobi','verbose',0),parameters_in,solutions_out);

%% RECEDING HORIZON CONTROL
for N = 1:numInterp*numSamples
    tic
    %% ESTIMATE HAND TRAJECTORY
    m = size(Xtargets,2); % number of targets
    n = 2;                % dimension of state
    reachingL = false;    % Assume not reaching
    reachingR = false;    % Assume not reaching
    
    % Sample Region (LEFT)
    leftSample = (leftHandStartSample)*numInterp + (N-1);
    numPrevSamples = numInterp*3;
    leftStartSample = leftSample - numPrevSamples;
    xh_partial_L = handPosInterp(leftStartSample:leftSample,:);   
    xhL = [xhL handPosInterp(leftSample,:)'];
    
    % Sample Region (RIGHT)
    rightSample = (rightHandStartSample)*numInterp + (N-1);
    rightStartSample = rightSample - numPrevSamples;
    xh_partial_R = handPosInterp(rightStartSample:rightSample,:);   
    xhR = [xhR handPosInterp(rightSample,:)'];
    
    %% Determine if reaching 
    %(LEFT)
    vh_partial_L = diff(xh_partial_L,1,1)./dt;
    if (vh_partial_L(end,2) > reachVelThresh)
        % Reaching
        reachingL = true;
    else
        % Not reaching
        reachingL = false;
    end
    
    %(RIGHT)
    vh_partial_R = diff(xh_partial_R,1,1)./dt;
    if (vh_partial_R(end,2) > reachVelThresh)
        % Reaching
        reachingR = true;
    else
        % Not reaching
        reachingR = false;
    end
    
    %% Determine Weights 
    % (LEFT)
    if (reachingL)
        emin = inf; % minimum error found
        for i = 1:m

            xt = Xtargets(:,i);    % Target position 

            % Solve optimal polynomial
            [px_, py_, x_, y_, t_, J_, e_] = getPolyEst2D(xh_partial_L,xt,dt);
            if e_ < emin
                % Assign our variables
                px = px_;
                py = py_;
                x = x_;
                y = y_;
                t = t_;
                J = J_;
                e = e_;
                m_min = i;

                % update min error
                emin = e;
            end
        end
        
        % Get weights
        tpred_L = 0:dt:dt*(T);
        xhand_L = [polyval(px,tpred_L);polyval(py,tpred_L)];  % Predicted hand trajectory
        vhand_L = diff(xhand_L')'./dt;                      % Predicted hand velocity
        w_L = zeros(T,m);
        wxt_L = zeros(T,n);

        for i = 1:T
            w_L(i,:) = getWeights(Xtargets, xhand_L(:,i), vhand_L(:,i));
            wxt_L(i,:) = sum((w_L(i,:).*Xtargets),2)';
        end
    else
        % If not reaching, assume weights are constant over horizon
        for i = 1:T
            w_L(i,:) = getWeights(Xtargets, handPosInterp(leftSample,:)', [0;0]);
            wxt_L(i,:) = sum((w_L(i,:).*Xtargets),2)';
        end
    end
    
    % (RIGHT)
    if (reachingR)
        emin = inf; % minimum error found
        for i = 1:m

            xt = Xtargets(:,i);    % Target position 

            % Solve optimal polynomial
            [px_, py_, x_, y_, t_, J_, e_] = getPolyEst2D(xh_partial_R,xt,dt);
            if e_ < emin
                % Assign our variables
                px = px_;
                py = py_;
                x = x_;
                y = y_;
                t = t_;
                J = J_;
                e = e_;
                m_min = i;

                % update min error
                emin = e;
            end
        end
        % Get weights
        tpred_R = 0:dt:dt*(T);
        xhand_R = [polyval(px,tpred_R);polyval(py,tpred_R)];  % Predicted hand trajectory
        vhand_R = diff(xhand_R')'./dt;                      % Predicted hand velocity
        w_R = zeros(T,m);
        wxt_R = zeros(T,n);

        for i = 1:T
            w_R(i,:) = getWeights(Xtargets, xhand_R(:,i), vhand_R(:,i));
            wxt_R(i,:) = sum((w_R(i,:).*Xtargets),2)';
        end
    else
        % If not reaching, assume weights are constant over horizon
        for i = 1:T
            w_R(i,:) = getWeights(Xtargets, handPosInterp(rightSample,:)', [0;0]);
            wxt_R(i,:) = sum((w_R(i,:).*Xtargets),2)';
        end
    end
    
    %% Assign Weights
    for i = 1:T
        WXT{i} = [wxt_L(i,:) wxt_R(i,:)];
    end

    %% Optimize

    inputs = {xr0,WXT{:}};
    [solutions,diagnostics] = controller{inputs};    
    u = solutions{1};
    xr = solutions{2};
    
    if diagnostics == 1
        error('The problem is infeasible');
    end
    
    if (N~=1)
        avgExecutionTime = avgExecutionTime + toc;  
    end
    
    %% Simulate Dynamics
    stdnoise = 0.00;    % Simulate system noise
    sysnoise = stdnoise.*randn(4,1);
    xr0_old = xr0;
    xr0 = xr0_old + u(:,1)*dt + sysnoise;
    xr_actual = [xr_actual xr0];   % Update actual trajectory
    u_actual = [u_actual u(:,1)];
    
    % Add incremental cost
    thisCost = 0;
    for j = 1:m
        thisCost = thisCost + w_L(1,j)*norm(xr0_old(1:2) - Xtargets(:,j))^2 ...
                        + w_R(1,j)*norm(xr0_old(3:4) - Xtargets(:,j))^2;
    end
    thisCost = thisCost + r*u(:,1)'*u(:,1);
    pathCost = pathCost + thisCost;
end

% Outputs
pathCost
avgExecutionTime = avgExecutionTime/(numInterp*numSamples)

%%
figure;

% subplot(3,1,1)
f = scatter(xhL(1,:),xhL(2,:),20,'o'); hold on;
scatter(xhR(1,:),xhR(2,:),20,'o');
plot(Xtargets(1,:),Xtargets(2,:),'xk','MarkerSize',7);
plot(xr_actual(1,:),xr_actual(2,:),'-s');
plot(xr_actual(3,:),xr_actual(4,:),'-s');
rectangle('Position',[x1min x2min (x1max-x1min) (x2max-x2min)],...
                        'EdgeColor', 'k', 'LineStyle', ':');  %visualize bounds
                    
w = 8;
h = 7;
% Rectangle where current robot is
rectangle('Position',[xr_actual(1,end)-w/2 xr_actual(2,end)-h/2 w h],...
                        'EdgeColor', 'k', 'LineStyle', '-');  %visualize bounds
rectangle('Position',[xr_actual(3,end)-w/2 xr_actual(4,end)-h/2 w h],...
                        'EdgeColor', 'k', 'LineStyle', '-');  %visualize bounds
                    
% Circle where current hand is
rectangle('Position',[xhL(1,end)-6/2 xhL(2,end)-6/2 6 6],...
                        'EdgeColor', 'k', 'LineStyle', '-', 'Curvature', [1 1]);  %visualize bounds
rectangle('Position',[xhR(1,end)-6/2 xhR(2,end)-6/2 6 6],...
                        'EdgeColor', 'k', 'LineStyle', '-', 'Curvature', [1 1]);  %visualize bounds
                    
title(sprintf('Spatial Trajectory'));
xlabel('x cm');
ylabel('y cm');
legend('Left Hand Traj','Right Hand Traj','Targets', 'Left Robot Traj', 'Right Robot Traj')

% subplot(3,1,2)
% plot(u_actual(1,:)); hold on;
% plot(u_actual(2,:),'--'); hold on;
% title('Left Control');
% xlabel('Time Step');
% ylabel('cm/s');
% legend('ux','uy') 
% 
% subplot(3,1,3)
% plot(u_actual(3,:)); hold on;
% plot(u_actual(4,:),'--'); hold on;
% title('Right Control');
% xlabel('Time Step');
% ylabel('cm/s');
% legend('ux','uy') 

