function [reaching] = determineIfReaching(handHistory,reachThresh,dt)
%determineIfReaching Returns true if hand is leaving body at sped greater
%than reachThresh
% Author: E. Gonzalez
% Date: 6/20/18

% Estimate hand velocity using backwards difference
vh = diff(handHistory,1,1)./dt;

if (vh(end,2) > reachThresh)
    reaching = true;
else
    reaching = false;
end

end

