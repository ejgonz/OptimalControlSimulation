function [weights] = determineWeights(T,reaching,handHistory,Xtargets,dt)
%determineWeights determine weights over future horizon
%   T: Horizon length (steps)
%   handHistory: previous hand positions (2xk)
%   Xtargets: Matrix containing positions of targets [xt1 xt2 ... xtm] 2xm
%   dt: Timestep

% Number of targets
m = size(Xtargets,2);

if (reaching)
    % Initialize minimum error found
    emin = inf;
    for i = 1:m
        xt = Xtargets(:,i);    % Target position 

        % Solve optimal polynomial
        [px_, py_, x_, y_, t_, J_, e_] = getPolyEst2D(handHistory,xt,dt);
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
    tpred = 0:dt:dt*(T);
    xhandPred = [polyval(px,tpred);polyval(py,tpred)];  % Predicted hand trajectory
    vhandPred = diff(xhandPred')'./dt;                      % Predicted hand velocity
    w = zeros(T,m);
    wxt = zeros(T,2);

    for i = 1:T
        w(i,:) = getWeights(Xtargets, xhandPred(:,i), vhandPred(:,i));
        wxt(i,:) = sum((w(i,:).*Xtargets),2)';
    end
else
    % If not reaching, assume weights are constant over horizon
    for i = 1:T
        w(i,:) = getWeights(Xtargets, handHistory(1,:)', [0;0]);
        wxt(i,:) = sum((w(i,:).*Xtargets),2)';
    end
end

weights = wxt;
    
end

