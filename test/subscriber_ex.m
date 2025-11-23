clc; clear; close all;

node = ros2node('/matlab_subscriber_node');
subscriber = ros2subscriber(node, '/chatter', @myCallback);

fprintf("Completion of node and subscriber complete!\n")
fprintf("Waiting for messages from publisher...\n")

disp('Subsribing to /chatter topic. Press Ctrl+C to stop.');
pause(inf)   % Pause infinitely to allow subscriber to recieve message

function myCallback(msg)
disp(['Received: ', msg.data])
end

%clear subscriber;
%clear node;c