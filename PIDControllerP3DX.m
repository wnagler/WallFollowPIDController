% Program: Script implementing a PID to make Pioneer 3-DX bot follow a wall in V-REP
% Name: IMAT5121_6_PID.m
% Author: Wim Nagler
% Date: 16/11/2020
% simRemoteApi.start(19999)

%%%%%%%%%%%%%%%%%%%%  Program Control Parameters  %%%%%%%%%%%%%%%%%%%%%%%%%

endTime = 3500;           %Number of iterations the wall following loop should run

setPoint = 0.65;         %The desired distance from the wall the robot must follow

frontCorrection = 0.70;  %Correcting for different placement on chassis of front and side sensor  
                         %This helps making left turns smoother
minFrontDistance = 0.2;  %If the bot is this close to the wall it can no longer turn normally and risks getting stuck                         
    
wheelSpeed = 3;          %Constant speed of the left wheel

Kp = 15;                 %Proportional Gain
Ki = 0;                  %Integral Gain
Kd = 300;                %Derivative Gain

%%%%%%%%%%%%%%%%%%%%%  START PROGRAM  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Load V-Rep Library, close existing connection and create new sim
vrep=remApi('remoteApi');       
vrep.simxFinish(-1);            
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

% Initialisation PID Calculation parameters
error = 0;                  %Current Error
prevError = 0;              %Previous Error
intError = 0;               %Cumulative Error (eg. Integral)

if (clientID>-1)
    % If connection successful
    disp('Connected')
    
    % Create handles for required V-Rep objects
    [returnCode,left_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [returnCode,right_Motor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    [returnCode,front_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',vrep.simx_opmode_blocking);
    [returnCode,right_Sensor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor8',vrep.simx_opmode_blocking);
    
    % Initialise sensors
    [returnCodeF,detectionStateF,distanceFront,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_streaming);
    [returnCodeR,detectionStateR,distanceRight,~,~]=vrep.simxReadProximitySensor(clientID,right_Sensor,vrep.simx_opmode_streaming);
    
    % Drive straight to the wall until wall found
    while (norm(distanceFront)*frontCorrection >= setPoint & detectionStateF==1) | detectionStateF==0;  %AND is needed to make sure we detected something
        % Start Driving straight
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, wheelSpeed ,vrep.simx_opmode_blocking);
        [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, wheelSpeed ,vrep.simx_opmode_blocking);
        % Ping Front Sonar
        [returnCodeF,detectionStateF,distanceFront,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_buffer);
    end  

    %%%%%%%%%%%%%%%%%%%%% BEGIN - PID CONTROL IMPLEMENTATION %%%%%%%%%%%%%%%%%%%%%%%%%    
    for i=1:endTime
        % Ping sonars
        [returnCodeF,detectionStateF,distanceFront,~,~]=vrep.simxReadProximitySensor(clientID,front_Sensor,vrep.simx_opmode_buffer);
        [returnCodeR,detectionStateR,distanceRight,~,~]=vrep.simxReadProximitySensor(clientID,right_Sensor,vrep.simx_opmode_buffer);
        
        % Setting the Front Flag
        if (norm(distanceFront)*frontCorrection <= setPoint) & detectionStateF==1   % Something in Front => left turn
            frontFlag = true;
        else
            frontFlag = false;  % Nothing in Front => Continue
        end
       
        % Setting Right Flag
        if detectionStateR == 0
            rightFlag = false;      % Nothing was detected on the right => turn right
        else 
            rightFlag = true;       % It means something was detected on the right => follow wall
        end
        
        if frontFlag
            if norm(distanceFront)<minFrontDistance
                %Robot stuck, back out randomly
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, -rand ,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, -rand ,vrep.simx_opmode_blocking);
                pause(0.1);
            else    
                %turn left inplace
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, 0 ,vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, wheelSpeed ,vrep.simx_opmode_blocking);
            end
        elseif rightFlag
            % PID
            % Calculate the current error
            error = norm(distanceRight) - setPoint;
            % Update the cumulative error
            intError = intError+error;
            % Calculate the derivative
            derivative = error - prevError;
            % Update the previous Error
            prevError=error;
            % Calculate control function
            u_t = Kp*error + Ki*intError + Kd*derivative;
           
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, wheelSpeed ,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, wheelSpeed-u_t ,vrep.simx_opmode_blocking);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %Code to comment / uncomment to change set-point and evaluate
            %PID reaction/settlement time/etc...
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %if i == 150
            %    setPoint=0.75;
            %elseif i==200
            %    setPoint=0.75;
            %elseif i==300
            %    setPoint=0.65;    
            %end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                
        else
            % turn right inplace because the wall sloped away
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, wheelSpeed ,vrep.simx_opmode_blocking);
            [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, 0 ,vrep.simx_opmode_blocking);
         
        end
    end  
    
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, right_Motor, 0 ,vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID, left_Motor, 0 ,vrep.simx_opmode_blocking);
%%%%%%%%%%%%%%%%%%%%% END - PID CONTROL IMPLEMENTATION %%%%%%%%%%%%%%%%%%%%%%%%%
    % Stop the simulation
    vrep.simxFinish(-1);
 
else
    % Connection Failed
    disp('Failed connecting to remote API server')
end

% Call the destructor
vrep.delete();
