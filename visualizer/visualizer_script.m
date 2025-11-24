%% ########## LIVE VISUALIZER ###########

clc; clear; close all;

%% 1. Live Plot, Generating Node and DataLogger Class

ID=0;
ROBOT_NUM=2;
MAX_PLOTPOINT=100;
WINDOW_DURATION=10;
PLOT_YLIM=2;
BATCH_NUM=5;

% Live plot
h_v_ref_lines=cell(1, ROBOT_NUM);   % Reference v from PLANNER
h_w_ref_lines=cell(1, ROBOT_NUM);   % Reference w from PLANNER
h_v_cnt_lines=cell(1, ROBOT_NUM);   % Command v from GROUND STATION
h_w_cnt_lines=cell(1, ROBOT_NUM);   % Command w from GROUND STATION
h_v_act_lines=cell(1, ROBOT_NUM);   % Actual v from OPTITRACK
h_w_act_lines=cell(1, ROBOT_NUM);   % Acutal w from OPTITRACK

h_pos_ref_lines=cell(1, ROBOT_NUM);      % Reference x fromm PLANNER
h_theta_ref_lines=cell(1, ROBOT_NUM);    % Reference theta fromm PLANNER
h_pos_act_lines=cell(1, ROBOT_NUM);      % Actual x fromm PLANNER
h_theta_act_lines=cell(1, ROBOT_NUM);    % Actual theta fromm PLANNER

ax_lin=cell(ROBOT_NUM, 1);
ax_ang=cell(ROBOT_NUM, 1);
ax_theta=cell(ROBOT_NUM, 1);

figure("Name", "LivePlot")
for i=1:ROBOT_NUM
    % Position plot
    subplot(ROBOT_NUM, 4, 4*i-3)
    hold on
    h_pos_ref_lines{i}=animatedline('Color', 'r');
    h_pos_act_lines{i}=animatedline('Color', 'b');

    h_pos_ref_lines{i}.MaximumNumPoints=MAX_PLOTPOINT;
    h_pos_act_lines{i}.MaximumNumPoints=MAX_PLOTPOINT;
    hold off
    xlabel("x [m]"); ylabel("y [m]"); xlim([-10, 10]); ylim([-10, 10]); axis equal;
    title(sprintf("Pos., Robot %d", i))
    
    % Heading plot
    subplot(ROBOT_NUM, 4, 4*i-2)
    ax_theta{i}=gca; ax_theta{i}.XLimMode='manual'; ax_theta{i}.YLimMode='auto';
    hold on
    h_theta_ref_lines{i}=animatedline('Color', 'r');
    h_theta_act_lines{i}=animatedline('Color', 'b');

    h_theta_ref_lines{i}.MaximumNumPoints=MAX_PLOTPOINT;
    h_theta_act_lines{i}.MaximumNumPoints=MAX_PLOTPOINT;
    hold off
    xlabel("Time [sec]"); ylabel("Heading [deg]");
    ylim([-180, +180]);
    title(sprintf("Heading, Robot %d", i))

    % Linear Velocity Plot
    subplot(ROBOT_NUM, 4, 4*i-1)
    ax_lin{i}=gca; ax_lin{i}.XLimMode='manual'; ax_lin{i}.YLimMode='auto';
    hold on
    h_v_ref_lines{i}=animatedline('Color', 'r');
    h_v_cnt_lines{i}=animatedline('Color', 'g');
    h_v_act_lines{i}=animatedline('Color', 'b');

    h_v_ref_lines{i}.MaximumNumPoints=MAX_PLOTPOINT;
    h_v_cnt_lines{i}.MaximumNumPoints=MAX_PLOTPOINT;
    h_v_act_lines{i}.MaximumNumPoints=MAX_PLOTPOINT;
    hold off
    xlabel("Time [sec]"); ylabel("v [m/s]");
    %xlim([0, 60]);   % Use moving window
    ylim([-PLOT_YLIM, PLOT_YLIM]);
    title(sprintf("Lin. Vel., Robot %d", i))

    % Angular Velocity Plot
    subplot(ROBOT_NUM, 4, 4*i)
    ax_ang{i}=gca; ax_ang{i}.XLimMode='manual'; ax_ang{i}.YLimMode='auto';
    hold on
    h_w_ref_lines{i}=animatedline('Color', 'r');
    h_w_cnt_lines{i}=animatedline('Color', 'g');
    h_w_act_lines{i}=animatedline('Color', 'b');

    h_w_ref_lines{i}.MaximumNumPoints=MAX_PLOTPOINT;
    h_w_cnt_lines{i}.MaximumNumPoints=MAX_PLOTPOINT;
    h_w_act_lines{i}.MaximumNumPoints=MAX_PLOTPOINT;
    hold off
    xlabel("Time [sec]"); ylabel("\omega [rad/s]");
    %xlim([0, 60]);   % Use moving window
    ylim([-PLOT_YLIM, PLOT_YLIM]);
    title(sprintf("Ang. Vel., Robot %d", i))

    sgtitle("Live Plot ({\color{red}Red=Ref.}, {\color{green}Green=Control}, {\color{blue}Blue=Actual})")
end

% Visualizer Node
node=ros2node('/matlab_visualizer_node', ID);

% DataLogger Class Object
obj=DataLogger(node, ROBOT_NUM);

fprintf('Subsribing to topics...\n')

i_v_ref=zeros(ROBOT_NUM, 1);
i_v_cnt=zeros(ROBOT_NUM, 1);
i_v_act=zeros(ROBOT_NUM, 1);
i_pos_ref=zeros(ROBOT_NUM, 1);
i_pos_act=zeros(ROBOT_NUM, 1);

cont_while_loop=true;
lastDraw=tic;
lastMessage=[];
t_temp=0;
while cont_while_loop
    for robot_idx=1:ROBOT_NUM
        % Plot reference velocities
        if length(obj.datalog_vel_ref{robot_idx}.t)>=i_v_ref(robot_idx)+BATCH_NUM
            i_v_ref(robot_idx)=i_v_ref(robot_idx)+BATCH_NUM;
            
            addpoints(h_v_ref_lines{robot_idx},...
                obj.datalog_vel_ref{robot_idx}.t(i_v_ref(robot_idx)-BATCH_NUM+1:i_v_ref(robot_idx)),...
                obj.datalog_vel_ref{robot_idx}.v(i_v_ref(robot_idx)-BATCH_NUM+1:i_v_ref(robot_idx)));

            addpoints(h_w_ref_lines{robot_idx},...
                obj.datalog_vel_ref{robot_idx}.t(i_v_ref(robot_idx)-BATCH_NUM+1:i_v_ref(robot_idx)),...
                obj.datalog_vel_ref{robot_idx}.w(i_v_ref(robot_idx)-BATCH_NUM+1:i_v_ref(robot_idx)));

            lastMessage=tic;
        end

        % Plot control velocities.
        if length(obj.datalog_vel_cnt{robot_idx}.t)>=i_v_cnt(robot_idx)+BATCH_NUM
            i_v_cnt(robot_idx)=i_v_cnt(robot_idx)+BATCH_NUM;
            

            addpoints(h_v_cnt_lines{robot_idx},...
                obj.datalog_vel_cnt{robot_idx}.t(i_v_cnt(robot_idx)-BATCH_NUM+1:i_v_cnt(robot_idx)),...
                obj.datalog_vel_cnt{robot_idx}.v(i_v_cnt(robot_idx)-BATCH_NUM+1:i_v_cnt(robot_idx)));

            addpoints(h_w_cnt_lines{robot_idx},...
                obj.datalog_vel_cnt{robot_idx}.t(i_v_cnt(robot_idx)-BATCH_NUM+1:i_v_cnt(robot_idx)),...
                obj.datalog_vel_cnt{robot_idx}.w(i_v_cnt(robot_idx)-BATCH_NUM+1:i_v_cnt(robot_idx)));

        end

        % Plot actual velocities
        if length(obj.datalog_vel_act{robot_idx}.t)>=i_v_act(robot_idx)+BATCH_NUM
            i_v_act(robot_idx)=i_v_act(robot_idx)+BATCH_NUM;
            
            addpoints(h_v_act_lines{robot_idx},...
                obj.datalog_vel_act{robot_idx}.t(i_v_act(robot_idx)-BATCH_NUM+1:i_v_act(robot_idx)),...
                obj.datalog_vel_act{robot_idx}.v(i_v_act(robot_idx)-BATCH_NUM+1:i_v_act(robot_idx)));

            addpoints(h_w_act_lines{robot_idx},...
                obj.datalog_vel_act{robot_idx}.t(i_v_act(robot_idx)-BATCH_NUM+1:i_v_act(robot_idx)),...
                obj.datalog_vel_act{robot_idx}.w(i_v_act(robot_idx)-BATCH_NUM+1:i_v_act(robot_idx)));
        end

        % Plot reference positions and headings
        if length(obj.datalog_pos_ref{robot_idx}.t)>=i_pos_ref(robot_idx)+BATCH_NUM
            i_pos_ref(robot_idx)=i_pos_ref(robot_idx)+BATCH_NUM;

            addpoints(h_pos_ref_lines{robot_idx},...
                obj.datalog_pos_ref{robot_idx}.x(i_pos_ref(robot_idx)-BATCH_NUM+1:i_pos_ref(robot_idx)),...
                obj.datalog_pos_ref{robot_idx}.y(i_pos_ref(robot_idx)-BATCH_NUM+1:i_pos_ref(robot_idx)));
            addpoints(h_theta_ref_lines{robot_idx},...
                obj.datalog_pos_ref{robot_idx}.t(i_pos_ref(robot_idx)-BATCH_NUM+1:i_pos_ref(robot_idx)),...
                rad2deg(obj.datalog_pos_ref{robot_idx}.theta(i_pos_ref(robot_idx)-BATCH_NUM+1:i_pos_ref(robot_idx))));
        end

        % Plot actual positions and headings
        if length(obj.datalog_pos_act{robot_idx}.t)>=i_pos_act(robot_idx)+BATCH_NUM
            i_pos_act(robot_idx)=i_pos_act(robot_idx)+BATCH_NUM;
            
            addpoints(h_pos_act_lines{robot_idx},...
                obj.datalog_pos_act{robot_idx}.x(i_pos_act(robot_idx)-BATCH_NUM+1:i_pos_act(robot_idx)),...
                obj.datalog_pos_act{robot_idx}.y(i_pos_act(robot_idx)-BATCH_NUM+1:i_pos_act(robot_idx)));
            addpoints(h_theta_act_lines{robot_idx},...
                obj.datalog_pos_act{robot_idx}.t(i_pos_act(robot_idx)-BATCH_NUM+1:i_pos_act(robot_idx)),...
                rad2deg(obj.datalog_pos_act{robot_idx}.theta(i_pos_act(robot_idx)-BATCH_NUM+1:i_pos_act(robot_idx))));
        end
    end

    if toc(lastDraw)>=0.05
        for i=1:ROBOT_NUM
            ax_lin{i}.XLim=[t_temp-WINDOW_DURATION, t_temp];
            ax_ang{i}.XLim=[t_temp-WINDOW_DURATION, t_temp];
            ax_theta{i}.XLim=[t_temp-WINDOW_DURATION, t_temp];
        end
        drawnow limitrate
        lastDraw=tic;

        if ~isempty(obj.datalog_vel_cnt{1}.t)
            t_temp=obj.datalog_vel_cnt{1}.t(end);
        end
    end

    if ~isempty(lastMessage) && toc(lastMessage)>=10
        cont_while_loop=false;
    end
end

fprintf("\n")
fprintf("Terminating Subscription and live plot.\n\n")

%% 2. Showing Results Post-Experiment
