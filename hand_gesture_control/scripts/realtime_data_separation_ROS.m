% 
% *********************************************************************************
% * Author: Gorkem Anil Al
% * Email: gga31@bath.ac.uk
% * Date: 6-March-2020
% *
% * University of Bath
% * Multimodal Interaction and Robotic Active Perception (inte-R-action) Lab
% * Centre for Autonomous Robotics (CENTAUR)
% * Department of Electronics and Electrical Engineering
% *
% * Description:
% *
% *********************************************************************************
% 
%%

% shutdown any existing instance of ROS
rosshutdown

%First initialize the arduino-matlab connection
delete(instrfind)
s=serial('COM15','BaudRate',9600); %start serial port to arduino
s.InputBufferSize = 2000; % Set the input buffer size to 2000 bytes.
s.ReadAsyncMode = 'continuous';
fopen(s); %open port
points = 200;


% Load trained network model
load('net500x52.mat');

data1 = zeros(1,26);
data2 = zeros(1,26);
i = 1;
reply = '';

%Secondly, initialize ROS-Matlab connection
rosinit('http://192.168.1.11:11311')

gesturePub = rospublisher('/gesture', 'std_msgs/String');

pause(2)

gestureMsg = rosmessage(gesturePub);

while (s.Status == 'open') 
    pause(2) 
    C =fscanf(s); 
    commas = strfind(C,',');    
    data = str2num(C);

    [m,n] = size(data);
    
    if (n > 2 && ~isempty(commas))
        data1 = str2num(C(1:commas(1)-1));
        data2 = str2num(C(commas(1):end));

        % set the size of the array
        [a,b] = size(data1);

        if (b < 26)
            data1(1,26) = 0;
        else
            z = b - 26; %if b > 320, find the z value and trim the last z value from ra_data matrix
            data1 = data1(1:end-z);                    
        end

        [c,d] = size(data2);
    
        if (d < 26)
            data2(1,26) = 0;
        else
            z2 = d - 26; %if b > 320, find the z value and trim the last z value from ra_data matrix
            data2 = data2(1:end-z2); 
        end

        concatenate_data = cat(2,data1, data2);     
        outputs = net(concatenate_data');
        [maxValue, linearIndexesOfMaxes] = max(outputs(:));  %index of max vule in the output matrix 
    
        if (linearIndexesOfMaxes == 1)
            reply = 'up'
        elseif (linearIndexesOfMaxes == 2)
            reply = 'down'
        elseif (linearIndexesOfMaxes == 3)
            reply = 'left'
        elseif (linearIndexesOfMaxes == 4)
            reply = 'right'
        else    
            reply = 'error'
        end

        %sending the message to the ROS master
        gestureMsg.Data = reply;
        send(gesturePub,gestureMsg)
        pause(2)
    end
    
    if(s.Status ~= 'open')
        rosshutdown  
        break;
    end
end

% shutdown any existing instance of ROS
rosshutdown

