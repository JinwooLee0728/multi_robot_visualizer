clc; clear; close all;

node = ros2node('/matlab_publisher_node');
publisher = ros2publisher(node, '/chatter', 'std_msgs/String');
msg = ros2message('std_msgs/String');

keyBoardInput=input('Start publishing? ... [y/n] ', 's');
if ~strcmp(keyBoardInput, 'y')
    fprintf("Terminating node\n")
    return
end

i=1;
while true
    msg.data = ['Hello from MATLAB, message number: ', num2str(i)];
    send(publisher, msg);
    disp(['Published: ' msg.data]);
    i=i+1;
    pause(1)
end

%clear publisher
%clear node