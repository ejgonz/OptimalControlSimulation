function [uRight, uLeft] = runDualMPC(controller, T, robotPosLeft, robotPosRight, handHistoryLeft, handHistoryRight, Xtargets, dt)
%runDualMPC Returns optimal control for left and right robots based
% on given optimal controller and number of previous hand points. 
%   controller: optimal controller formed from createDualOptimizer
%   robotPosLeft/Right: current robot position
%   handHistoryLeft: previous left hand positions (2xk)
%   handHistoryRight: previous right hand positions (2xk)
%   uLeft: x,y commanded velocities for left-hand robot (2x1)
%   uRight: x,y commanded velocities for right-hand robot (2x1)
%   T: future horizon (steps)
% Author: E. Gonzalez
% Date: 6/20/18
% Execution Time: TBD

% Determine if Reaching
reachThresh = 10; % min speed moving away from body to determine reaching [cm/s]
rightReaching = determineIfReaching(handHistoryRight,reachThresh,dt);
leftReaching = determineIfReaching(handHistoryLeft,reachThresh,dt);

% Determine Weights
rightWeights = determineWeights(T,rightReaching,handHistoryRight,Xtargets,dt);
leftWeights = determineWeights(T,leftReaching,handHistoryLeft,Xtargets,dt);

%Comnbine Weights
AllWeights = cell(1,T);
for i = 1:T
    AllWeights{i} = [leftWeights(i,:) rightWeights(i,:)];
end

% Get Optimal Control
xr0 = [robotPosLeft; robotPosRight];
inputs = {xr0,AllWeights{:}};
u_opt = controller{inputs}';

uLeft = u_opt{1}(1:2);
uRight = u_opt{1}(3:4);

end

