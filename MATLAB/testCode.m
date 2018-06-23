clear all;
close all;
load('Dataset1.mat');
Xtargets = Xtargets([1 3],:);   % Grab only x and y of target positions

T = 6;
x1max = 5; x1min = -55;
x2max = 7.5; x2min = -20;
xLimits = [x1min; x2min; x1max; x2max];
maxSpeed = 60;
r = 0;
dt = 0.02;
safetyDist = 6;

robotPosLeft = [-35;-10];
robotPosRight = [-20;0];
% Interpolate data points (simulate sample time of 0.01)
numInterp = 1;
handPosInterpX = interp1q((1:length(handPos))',handPos(:,1),(1:1/numInterp:length(handPos))');
handPosInterpY = interp1q((1:length(handPos))',handPos(:,3),(1:1/numInterp:length(handPos))');
handPosInterp = [handPosInterpX handPosInterpY];

leftSample = (471)*numInterp;
numPrevSamples = numInterp*3;
leftStartSample = leftSample - numPrevSamples;
handHistoryLeft = handPosInterp(leftStartSample:leftSample,:); 

rightSample = (494)*numInterp;
rightStartSample = rightSample - numPrevSamples;
handHistoryRight = handPosInterp(rightStartSample:rightSample,:);   


tic
controller = createDualOptimizer(T, xLimits, maxSpeed, r, dt, safetyDist);
toc
%%
tic
[uRight, uLeft] = runDualMPC(controller, T, robotPosLeft, robotPosRight, handHistoryLeft, handHistoryRight,Xtargets,dt);
toc