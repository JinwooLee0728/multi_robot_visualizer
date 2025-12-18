%% ########## LIVE VISUALIZER ###########

clc; clear; close all;

%% 1. Live Plot, Generating Node and DataLogger Class

ID=7;
ROBOT_NUM=2;
MAX_PLOTPOINT=2000;
WINDOW_DURATION=10;
PLOT_YLIM=3;
SAMPLE_INTV=1;

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

    h_pos_ref_lines{i}.MaximumNumPoints=MAX_PLOTPOINT*100;
    h_pos_act_lines{i}.MaximumNumPoints=MAX_PLOTPOINT*100;
    hold off
    xlabel("x [m]"); ylabel("y [m]"); xlim([-5, 5]); ylim([-5, 5]); grid on;
    % axis equal;
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
    ylim([-0.5, 0.5]);
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

fprintf('Subscribing to topics...\n')

i_v_ref=zeros(ROBOT_NUM, 1);
i_v_cnt=zeros(ROBOT_NUM, 1);
i_v_act=zeros(ROBOT_NUM, 1);
i_pos_ref=zeros(ROBOT_NUM, 1);
i_pos_act=zeros(ROBOT_NUM, 1);

cont_while_loop=true;
lastDraw=tic;
lastMessage=[];
t_temp=0;
time_now=[];
while cont_while_loop
    for robot_idx=1:ROBOT_NUM
        % Plot reference velocities
        if length(obj.datalog_vel_ref{robot_idx}.t)>=i_v_ref(robot_idx)+SAMPLE_INTV
            i_v_ref(robot_idx)=i_v_ref(robot_idx)+SAMPLE_INTV;
            if ~isempty(time_now)

                %{
                time_arr=obj.datalog_vel_ref{robot_idx}.t(:, i_v_ref(robot_idx));
                mod_arr=zeros(1, length(time_arr));
                for i=1:length(time_arr)
                    mod_arr(i)=abs(time_arr(i)-time_now);
                end
                [~, closest_idx]=min(mod_arr);
                %}

                addpoints(h_v_ref_lines{robot_idx},...
                    obj.datalog_vel_ref{robot_idx}.t(i_v_ref(robot_idx)),...
                    obj.datalog_vel_ref{robot_idx}.v(i_v_ref(robot_idx)));

                addpoints(h_w_ref_lines{robot_idx},...
                    obj.datalog_vel_ref{robot_idx}.t(i_v_ref(robot_idx)),...
                    obj.datalog_vel_ref{robot_idx}.w(i_v_ref(robot_idx)));
            end

            %lastMessage=tic;
        end

        % Plot control velocities.
        if length(obj.datalog_vel_cnt{robot_idx}.t)>=i_v_cnt(robot_idx)+SAMPLE_INTV
            i_v_cnt(robot_idx)=i_v_cnt(robot_idx)+SAMPLE_INTV;

            addpoints(h_v_cnt_lines{robot_idx},...
                obj.datalog_vel_cnt{robot_idx}.t(i_v_cnt(robot_idx)),...
                obj.datalog_vel_cnt{robot_idx}.v(i_v_cnt(robot_idx)));

            addpoints(h_w_cnt_lines{robot_idx},...
                obj.datalog_vel_cnt{robot_idx}.t(i_v_cnt(robot_idx)),...
                obj.datalog_vel_cnt{robot_idx}.w(i_v_cnt(robot_idx)));

            time_now=obj.datalog_vel_cnt{robot_idx}.t(i_v_cnt(robot_idx));

            lastMessage=tic;

        end

        % Plot actual velocities
        if length(obj.datalog_vel_act{robot_idx}.t)>=i_v_act(robot_idx)+SAMPLE_INTV
            i_v_act(robot_idx)=i_v_act(robot_idx)+SAMPLE_INTV;

            addpoints(h_v_act_lines{robot_idx},...
                obj.datalog_vel_act{robot_idx}.t(i_v_act(robot_idx)),...
                obj.datalog_vel_act{robot_idx}.v(i_v_act(robot_idx)));

            addpoints(h_w_act_lines{robot_idx},...
                obj.datalog_vel_act{robot_idx}.t(i_v_act(robot_idx)),...
                obj.datalog_vel_act{robot_idx}.w(i_v_act(robot_idx)));
        end

        % Plot reference positions and headings
        if length(obj.datalog_pos_ref{robot_idx}.t)>=i_pos_ref(robot_idx)+SAMPLE_INTV
            i_pos_ref(robot_idx)=i_pos_ref(robot_idx)+SAMPLE_INTV;

            addpoints(h_pos_ref_lines{robot_idx},...
                obj.datalog_pos_ref{robot_idx}.x(i_pos_ref(robot_idx)),...
                obj.datalog_pos_ref{robot_idx}.y(i_pos_ref(robot_idx)));
            addpoints(h_theta_ref_lines{robot_idx},...
                obj.datalog_pos_ref{robot_idx}.t(i_pos_ref(robot_idx)),...
                rad2deg(obj.datalog_pos_ref{robot_idx}.theta(i_pos_ref(robot_idx))));
        end

        % Plot actual positions and headings
        if length(obj.datalog_pos_act{robot_idx}.t)>=i_pos_act(robot_idx)+SAMPLE_INTV
            i_pos_act(robot_idx)=i_pos_act(robot_idx)+SAMPLE_INTV;
            t_temp=obj.datalog_pos_act{robot_idx}.t(i_pos_act(robot_idx));

            addpoints(h_pos_act_lines{robot_idx},...
                obj.datalog_pos_act{robot_idx}.x(i_pos_act(robot_idx)),...
                obj.datalog_pos_act{robot_idx}.y(i_pos_act(robot_idx)));
            addpoints(h_theta_act_lines{robot_idx},...
                obj.datalog_pos_act{robot_idx}.t(i_pos_act(robot_idx)),...
                rad2deg(obj.datalog_pos_act{robot_idx}.theta(i_pos_act(robot_idx))));
        end
    end

    if toc(lastDraw)>=0.02
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

%clearvars -except ROBOT_NUM obj
robot_colors=orderedcolors("gem");

figure("Name", "Trajectory and Heading")
subplot(4, ROBOT_NUM, 1:3*ROBOT_NUM)
hold on
for i=1:ROBOT_NUM
    plot(obj.datalog_pos_ref{i}.x, obj.datalog_pos_ref{i}.y,...
        'Color', robot_colors(i, :), 'Linestyle', '--', 'DisplayName', sprintf('Ref. pos., Robot %d', i))
    plot(obj.datalog_pos_act{i}.x, obj.datalog_pos_act{i}.y,...
        'Color', robot_colors(i, :), 'DisplayName', sprintf('Act. pos., Robot %d', i))
end
hold off
xlabel("x [m]"); ylabel("y [m]"); axis equal; grid on;
legend("Location", "northeast")
title("Robot Pos.")

for i=1:ROBOT_NUM
    subplot(4, ROBOT_NUM, 3*ROBOT_NUM+i)
    hold on
    plot(obj.datalog_pos_ref{i}.t, rad2deg(obj.datalog_pos_ref{i}.theta),...
        'Color', 'r', 'DisplayName', sprintf('Ref. \theta, Robot %d', i))
    plot(obj.datalog_pos_act{i}.t, rad2deg(obj.datalog_pos_act{i}.theta),...
        'Color', 'b', 'DisplayName', sprintf('Act. \theta, Robot %d', i))
    hold off
    xlabel("t [s]"); ylabel("\theta [deg]");
    legend("Location", "northeast")
    title(sprintf('Robot %d Heading', i))
end
sgtitle("Trajectory and Heading")


figure("Name", "Velocities")
for i=1:ROBOT_NUM
    subplot(ROBOT_NUM, 2, 2*i-1)
    hold on
    plot(obj.datalog_vel_cnt{i}.t, obj.datalog_vel_cnt{i}.v, ...
        'Color', 'g', 'DisplayName', sprintf('Cnt. v, Robot %d', i))
    plot(obj.datalog_vel_act{i}.t, obj.datalog_vel_act{i}.v, ...
        'Color', 'b', 'DisplayName', sprintf('Act. v, Robot %d', i))
    plot(obj.datalog_vel_ref{i}.t, obj.datalog_vel_ref{i}.v, ...
        'Color', 'r', 'DisplayName', sprintf('Ref. v, Robot %d', i))
    %addpoints(h_v_act_lines{i}, obj.datalog_vel_ref{i}.t(end, end), 0);

    hold off
    xlabel("t [s]"); ylabel("v [m/s]");
    legend("Location", "northeast")
    title(sprintf('Robot %d v', i))

    subplot(ROBOT_NUM, 2, 2*i)
    hold on
    plot(obj.datalog_vel_cnt{i}.t, obj.datalog_vel_cnt{i}.w, ...
        'Color', 'g', 'DisplayName', sprintf('Cnt. w, Robot %d', i))
    plot(obj.datalog_vel_act{i}.t, obj.datalog_vel_act{i}.w, ...
        'Color', 'b', 'DisplayName', sprintf('Act. w, Robot %d', i))
    plot(obj.datalog_vel_ref{i}.t, obj.datalog_vel_ref{i}.w, ...
        'Color', 'r', 'DisplayName', sprintf('Ref. w, Robot %d', i))
    ylim([-5, 5]);
    %addpoints(h_w_act_lines{i}, obj.datalog_vel_ref{i}.t(end, end), 0);

    hold off
    xlabel("t [s]"); ylabel("w [rad/s]");
    legend("Location", "northeast")
    title(sprintf('Robot %d w', i))
end
sgtitle('Robot Velocities')
