% -- Main.m -- 
%   Main MATLAB script for executing multi robot control using real-time
%   data recieved from Unity over UDP
% Author: E. Gonzalez
% Date: 6/20/18
warning('off','gurobi:Deprecated')

% State variables
WAITING2INIT = 0;
WAITING4DATA = 1;
currentState = WAITING2INIT;

% Data headers (FROM Unity)
INITIALIZE_CMD = 120;
DATA_CMD = 121;
END_CMD = 122;

% Command headers (TO Unity)
READY_CMD = 220;
CONTROL_CMD = 221;

% Create UDP connection (one port for reading, another for receiving) 
port = udp('localhost', 8000, 'LocalPort', 8001, 'ByteOrder', 'littleEndian');
fopen(port);

fprintf("UDP Port is %s. \nWaiting for initialization request from Unity. \n", port.status);

running = true;
while running
    msg = double(readMessage(port));
    messageHeader = msg(1);
    
    switch messageHeader
        case INITIALIZE_CMD 
            disp("Initializing...");
            
            % Parse initialization parameters
            T       = msg(2);   % Horizon (steps)
            
            x1max   = msg(3); % Bounds (cm)
            x1min   = msg(4);
            x2max   = msg(5);
            x2min   = msg(6); 
            xLimits = [x1min; x2min; x1max; x2max];
            
            maxSpeed   = msg(7);   % Robot speed limit (cm/s)
            r          = msg(8);   % Control penalty
            dt         = msg(9);   % Timestep (seconds)
            safetyDist = msg(10);  % Collision avoidance margin (cm)
            
            numTargets = msg(11);  % Assemble matrix of target positions
            Xtargets = zeros(2,numTargets);
            for i = 1:numTargets
                Xtargets(:,i) = [msg(12 + 2*(i-1)); msg(13 + 2*(i-1))]; 
            end
            
            % Create optimizer
            controller = createDualOptimizer(T, xLimits, maxSpeed, r, dt, safetyDist);
            
            % Send confirmation
            sendMessage(READY_CMD, port);
            initialized = true;
            disp('Initialization complete');
            disp("Running...");
        case DATA_CMD

            if (initialized) 
                % Parse data
                robotPosLeft  = [msg(2); msg(3)]; % x,y positions
                robotPosRight = [msg(4); msg(5)];
                
                numPrevPoints = msg(6); % Number of points in hand history
                handHistoryLeft = zeros(2,numPrevPoints); 
                handHistoryRight = zeros(2,numPrevPoints); 
                for i = 1:numPrevPoints
                   % all left hand x,y sent sequentially then all right
                   % hand xy sent sequentially
                   
                   % Left hand history
                   handHistoryLeft(:,i) = [msg(7 + 2*(i-1)); msg(8 + 2*(i-1))];
                   
                   % Right hand history
                   handHistoryRight(:,i) = [msg(7 + 2*(i-1+numPrevPoints)); msg(8 + 2*(i-1+numPrevPoints))];
                end
                
                % Clear input buffer 
                % flushinput(port);
                
                uRight = [0;0]; uLeft = [0;0];
                [uRight, uLeft] = runDualMPC(controller, T, ...
                    robotPosLeft, robotPosRight, handHistoryLeft', ...
                    handHistoryRight',Xtargets,dt);     
                
                % Send control back to unity
                controlMsg = [CONTROL_CMD uRight' uLeft'];
                sendMessage(controlMsg, port);
            end
            
        case END_CMD
            % End transmission
            disp("Unity program terminated.");
            fclose(port);
            running = false;
   
        case -1
            % Do nothing. No message recieved.
    end
    
end

% -- readMessage --
%   Read message from port using UDP. Returns -1 if no message available. 
function msg = readMessage(port) 
   % Read incoming messages from LocalPort
   if (port.bytesAvailable)
       msg = fread(port,100,'single');
   else
       msg = -1;
   end
end

% -- sendMessage --
%   Send message to port using UDP. 
function sendMessage(msg, port) 
   fwrite(port,msg,'single'); 
end