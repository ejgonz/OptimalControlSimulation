function w = getWeights(Xtargets, xhand, vhand)
%GETWEIGHTS 
%   Returns vector of target weights. Based on Shigeta 2007.
%   Inputs
%       xhand: current hand (x,y) position [2x1]
%       vhand: current hand (vx,vy) velocity
%       Xtargets: matrix of target (x,y) positions [2xm]
%   Outputs
%       w: target weights [mx1]

%% COMPONENT 1: Distance
% Compute displacement between hand and each target
disp = Xtargets - xhand;
d = vecnorm(disp);
w1 = d.^(-3);
if sum(isinf(w1))
    % inf indicates hand is exactly at a target.
    % Make it a large number insted so MATLAB can handle it.
    w1 = 10000.*isinf(w1);  
end

%% COMPONENT 2: Bias Correction Factor
% Compute distance between hand and each target
m = size(Xtargets,2);   % number of targets
w2 = zeros(1,m);
for i = 1:m
    xt = Xtargets(:,i);
    w2(i) = sum(vecnorm(repmat(xt,[1,m]) - Xtargets));
end

%% COMPONENT 3: Velocity
% Moving towards object: weight more
% Moving away from object: weight less
% Moving perpendicular to object: weight 1
vh_dot_disp = vhand'*disp;  % Compute dot products between vhand and displacement vectors [1xm]
alpha = 0.01;               % Scaling factor TODO tweak
w3max = 2;                 % Max value of this weight TODO tweak
w3 = max(0.5,min(exp(alpha*vh_dot_disp),w3max));

%% Multiply and Normalize Weights
w = (w1.*w2).*w3;
w = w./sum(w);
w = w'; % output as column vector
end

