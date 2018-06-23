function [controller] = createDualOptimizer(T, xLimits, maxSpeed, r, dt, safetyDist)
%CREATE DUAL OPTIMIZER Create custom controller for solving dual robot
% optimization problem. Uses YALMIP. Inputs are the initial conditions (x) and
% future weights (WXT).
%   T: Horizon (timesteps)
%   xLimits: State constraints [x1Min; x2Min; x1Max; x2Max] ()sm)
%   maxSpeed: Maximum robot speed (cm/s)
%   r: Control penalty term
%   dt: sample time
% Author: E. Gonzalez
% Date: 6/16/18
% Execution Time: 4.374006 seconds [6/20/18]

%% YALMIP FORMULATION
% Model data
nx = 4; % Number of states
nu = 4; % Number of inputs

% State constraints
%   Stack for 2 robots
x1min = xLimits(1);
x2min = xLimits(2);
x1max = xLimits(3);
x2max = xLimits(4);

% Control Constraint Variables
umax2 = [maxSpeed^2; maxSpeed^2];
C = [1 0 0 0; 0 0 1 0];
D = [0 1 0 0; 0 0 0 1];

% Optimization Variables
u = sdpvar(repmat(nu,1,T),repmat(1,1,T));
x = sdpvar(repmat(nx,1,T+1),repmat(1,1,T+1));
WXT = sdpvar(repmat(1,1,T),repmat(nx,1,T));

constraints = [];
objective = 0;
for k = 1:T
 objective = objective + x{k}'*x{k} - 2*WXT{k}*x{k} + r*u{k}'*u{k};
 
 % Dynamics
 constraints = [constraints, x{k+1} == x{k} + dt*u{k}];
 
 % Control Constraints
 constraints = [constraints, (C*u{k}).^2 + (D*u{k}).^2 <= umax2];
 
% State Constraints
 constraints = [constraints, x1min <= x{k}(1) <= x1max,...
                             x2min <= x{k}(2) <= x2max,...
                             x1min <= x{k}(3) <= x1max,...
                             x2min <= x{k}(4) <= x2max];

 % Collision Avoidance
 SafetyRegionRight = [x{k}(1) - x{k}(3) >= safetyDist];
 SafetyRegionLeft  = [x{k}(3) - x{k}(1) >= safetyDist];
 SafetyRegionAbove = [x{k}(2) - x{k}(4) >= safetyDist];
 SafetyRegionBelow = [x{k}(4) - x{k}(2) >= safetyDist];
 constraints = [constraints, SafetyRegionRight|SafetyRegionLeft|SafetyRegionAbove|SafetyRegionBelow];
end

parameters_in = {x{1},WXT{:}};
solutions_out = {u{:}};

controller = optimizer(constraints, objective,sdpsettings('solver','gurobi','verbose',0),parameters_in,solutions_out);
end

