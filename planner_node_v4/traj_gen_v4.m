%% ########## TRAJECTORY GENERATOR FOR LOW-LEVEL CONTROL TEST ##########

clc; clear; close all;
delete('x_ob_ref.m'); delete('y_ob_ref.m'); delete('theta_ob_ref.m')
delete('x_rb_ref.m'); delete('y_rb_ref.m'); delete('theta_rb_ref.m')
delete('v_rb_ref.m'); delete('omega_rb_ref.m');

%% 1. User-Defined Values

ROBOT_NUM=2;                        % Number of robots
ROBOT_REL_POS=zeros(ROBOT_NUM, 2);  % Positions of robots, in object body-fixed frame
ROBOT_REL_POS(1, :)=[1.0; +0.0];
ROBOT_REL_POS(2, :)=[-1.0, +0.0];

%OBJ_X_INIT=0;
%OBJ_Y_INIT=0;
OBJ_X_INIT=-1.492382;
OBJ_Y_INIT=-1.805996;
OBJ_THETA_INIT=deg2rad(0);

CHAR_V1=0.3;       % [m/s]
CHAR_OMEGA1=0.3;   % [rad/s]
CHAR_ANGLE1=pi/2;  % [rad]
CHAR_L1=3;         % [m]
CHAR_L2=1;         % [m]
T_END=20;          % [s]

N_CMD_LIST=20;    % Number of commands in one publish
DT=0.01;          % Time between each command in one publish

VIEW_TRAJ=true;   % Toggle to view plot of trajectory

ID=7;

%% 2. Test Trajectory Generation

fprintf('========== Trajectory Types ==========\n')
fprintf('1. Pure translation\n')
fprintf('2. Pure rotation\n')
fprintf('3. Circles\n')
fprintf('4. Translation + Rotation \n')
fprintf('5. Turning around a corner \n')
fprintf('6. Circles and straight lines \n')
fprintf('7. Ellipse \n')
fprintf('======================================\n\n')
traj_type=input('Choose trajectory type (input number) : ');

t=sym('t');
% Reference Trajectory of Object
x_ob_ref=sym('x_ob_ref'); y_ob_ref=sym('y_ob_ref'); theta_ob_ref=sym('theta_ob_ref');

% Reference Trajectory of Robots
x_rb_ref=sym('x_rb_ref', [ROBOT_NUM, 1]);
y_rb_ref=sym('y_rb_ref', [ROBOT_NUM, 1]);
theta_rb_ref=sym('theta_rb_ref', [ROBOT_NUM, 1]);

% Reference Velocities of Robots
dx_rb_ref=sym('dx_rb_ref', [ROBOT_NUM, 1]);
dy_rb_ref=sym('dy_rb_ref', [ROBOT_NUM, 1]);
v_rb_ref=sym('dy_rb_ref', [ROBOT_NUM, 1]);
omega_rb_ref=sym('omega_rb_ref', [ROBOT_NUM, 1]);

% Object trajectory
if traj_type==1
    x_ob_ref=piecewise(...
        t<=CHAR_L1/CHAR_V1, OBJ_X_INIT+t*CHAR_V1*cos(OBJ_THETA_INIT), ...
        t>CHAR_L1/CHAR_V1, OBJ_X_INIT+(CHAR_L1/CHAR_V1)*CHAR_V1*cos(OBJ_THETA_INIT));
    y_ob_ref=piecewise(...
        t<=CHAR_L1/CHAR_V1, OBJ_Y_INIT+t*CHAR_V1*sin(OBJ_THETA_INIT), ...
        t>CHAR_L1/CHAR_V1, OBJ_Y_INIT+(CHAR_L1/CHAR_V1)*CHAR_V1*sin(OBJ_THETA_INIT));
    theta_ob_ref=OBJ_THETA_INIT+(1e-10)*t;

    T_END=CHAR_L1/CHAR_V1;   % Recalculated T_END

elseif traj_type==2
    x_ob_ref=OBJ_X_INIT+(1e-10)*t;
    y_ob_ref=OBJ_Y_INIT+(1e-10)*t;
    theta_ob_ref=OBJ_THETA_INIT+CHAR_OMEGA1*t;

elseif traj_type==3
    temp=atan(-1/tan(OBJ_THETA_INIT));
    x_ob_ref=OBJ_X_INIT+CHAR_L2*cos(CHAR_OMEGA1*t+temp);
    y_ob_ref=OBJ_X_INIT+CHAR_L2*sin(CHAR_OMEGA1*t+temp);
    theta_ob_ref=OBJ_THETA_INIT+CHAR_OMEGA1*t;

    T_END=2*pi/CHAR_OMEGA1;   % Recalculated T_END

    clearvars temp;

elseif traj_type==4
    x_ob_ref=piecewise(...
        t<=CHAR_L1/CHAR_V1, OBJ_X_INIT+t*CHAR_V1*cos(OBJ_THETA_INIT), ...
        t>CHAR_L1/CHAR_V1, OBJ_X_INIT+(CHAR_L1/CHAR_V1)*CHAR_V1*cos(OBJ_THETA_INIT));
    y_ob_ref=piecewise(...
        t<=CHAR_L1/CHAR_V1, OBJ_Y_INIT+t*CHAR_V1*sin(OBJ_THETA_INIT), ...
        t>CHAR_L1/CHAR_V1, OBJ_Y_INIT+(CHAR_L1/CHAR_V1)*CHAR_V1*sin(OBJ_THETA_INIT));
    theta_ob_ref=OBJ_THETA_INIT+(CHAR_OMEGA1)*t;

    T_END=CHAR_L1/CHAR_V1;   % Recalculated T_END
elseif traj_type==5
    temp1=CHAR_L1/CHAR_V1;
    temp2=temp1+CHAR_ANGLE1/CHAR_OMEGA1;
    temp3=temp2+CHAR_L2/CHAR_V1;

    x_ob_ref=piecewise(...
        t<=temp1, OBJ_X_INIT+t*CHAR_V1*cos(OBJ_THETA_INIT), ...
        temp1<t<=temp2, OBJ_X_INIT+temp1*CHAR_V1*cos(OBJ_THETA_INIT),...
        temp2<t, (OBJ_X_INIT+temp1*CHAR_V1*cos(OBJ_THETA_INIT)) + (t-temp2)*CHAR_V1*cos(OBJ_THETA_INIT+CHAR_ANGLE1));

    y_ob_ref=piecewise(...
        t<=temp1, OBJ_Y_INIT+t*CHAR_V1*sin(OBJ_THETA_INIT), ...
        temp1<t<=temp2, OBJ_Y_INIT+temp1*CHAR_V1*sin(OBJ_THETA_INIT),...
        temp2<t, (OBJ_Y_INIT+temp1*CHAR_V1*sin(OBJ_THETA_INIT)) + (t-temp2)*CHAR_V1*sin(OBJ_THETA_INIT+CHAR_ANGLE1));

    theta_ob_ref=piecewise(...
        t<=temp1, OBJ_THETA_INIT, ...
        temp1<t<=temp2, OBJ_THETA_INIT+CHAR_OMEGA1*(t-temp1),...
        temp2<t, OBJ_THETA_INIT+CHAR_ANGLE1);

    T_END=temp3;

    clearvars temp1 temp2 temp3

elseif traj_type==6
    temp1=CHAR_L1/CHAR_V1;
    temp2=temp1+pi/CHAR_OMEGA1;
    temp3=temp2+CHAR_L1/CHAR_V1;
    temp4=temp3+pi/CHAR_OMEGA1;

    temp=atan(-1/tan(OBJ_THETA_INIT));

    x_ob_ref(t)=piecewise(...
        t<=temp1, OBJ_X_INIT+t*CHAR_V1*cos(OBJ_THETA_INIT), ...
        temp1<t<=temp2, OBJ_X_INIT+temp1*CHAR_V1*cos(OBJ_THETA_INIT)-CHAR_L2*cos(temp)+CHAR_L2*cos(CHAR_OMEGA1*(t-temp1)+temp),...
        temp2<t<=temp3, OBJ_X_INIT+temp1*CHAR_V1*cos(OBJ_THETA_INIT)-CHAR_L2*cos(temp)+CHAR_L2*cos(CHAR_OMEGA1*(temp2-temp1)+temp)...
        +(t-temp2)*CHAR_V1*cos(OBJ_THETA_INIT+pi),...
        temp3<t, OBJ_X_INIT+temp1*CHAR_V1*cos(OBJ_THETA_INIT)-CHAR_L2*cos(temp)+CHAR_L2*cos(CHAR_OMEGA1*(temp2-temp1)+temp)...
        +(temp3-temp2)*CHAR_V1*cos(OBJ_THETA_INIT+pi) - CHAR_L2*cos(temp+pi)+CHAR_L2*cos(CHAR_OMEGA1*(t-temp3)+temp+pi));

    y_ob_ref(t)=piecewise(...
        t<=temp1, OBJ_Y_INIT+t*CHAR_V1*sin(OBJ_THETA_INIT), ...
        temp1<t<=temp2, OBJ_Y_INIT+temp1*CHAR_V1*sin(OBJ_THETA_INIT)-CHAR_L2*sin(temp)+CHAR_L2*sin(CHAR_OMEGA1*(t-temp1)+temp),...
        temp2<t<=temp3, OBJ_Y_INIT+temp1*CHAR_V1*sin(OBJ_THETA_INIT)-CHAR_L2*sin(temp)+CHAR_L2*sin(CHAR_OMEGA1*(temp2-temp1)+temp)...
        +(t-temp2)*CHAR_V1*sin(OBJ_THETA_INIT+pi),...
        temp3<t, OBJ_Y_INIT+temp1*CHAR_V1*sin(OBJ_THETA_INIT)-CHAR_L2*sin(temp)+CHAR_L2*sin(CHAR_OMEGA1*(temp2-temp1)+temp)...
        +(temp3-temp2)*CHAR_V1*sin(OBJ_THETA_INIT+pi) - CHAR_L2*sin(temp+pi)+CHAR_L2*sin(CHAR_OMEGA1*(t-temp3)+temp+pi));

    theta_ob_ref(t)=piecewise(...
        t<=temp1, OBJ_THETA_INIT, ...
        temp1<t<=temp2, OBJ_THETA_INIT+CHAR_OMEGA1*(t-temp1),...
        temp2<t<=temp3, OBJ_THETA_INIT+pi,...
        temp3<t, OBJ_THETA_INIT+pi+CHAR_OMEGA1*(t-temp3));

    %x_ob_ref(t)=x_ob_ref(mod(t, temp4));
    %y_ob_ref(t)=y_ob_ref(mod(t, temp4));
    %theta_ob_ref(t)=theta_ob_ref(mod(t, temp4));

    T_END=temp4;

elseif traj_type==7
    temp=atan(-1/tan(OBJ_THETA_INIT));
    x_ob_ref=OBJ_X_INIT+CHAR_L1*cos(CHAR_OMEGA1*t+temp);
    y_ob_ref=OBJ_X_INIT+CHAR_L2*sin(CHAR_OMEGA1*t+temp);
    theta_ob_ref=OBJ_THETA_INIT+CHAR_OMEGA1*t;

    T_END=2*pi/CHAR_OMEGA1;   % Recalculated T_END

    clearvars temp;

else
    fprintf("ERROR: Wrong trajectory type input\n")
    return
end

% Robot trajectory
for i=1:ROBOT_NUM
    x_rb_ref(i)=x_ob_ref+cos(theta_ob_ref)*ROBOT_REL_POS(i, 1)-sin(theta_ob_ref)*ROBOT_REL_POS(i, 2);
    y_rb_ref(i)=y_ob_ref+sin(theta_ob_ref)*ROBOT_REL_POS(i, 1)+cos(theta_ob_ref)*ROBOT_REL_POS(i, 2);
        
    dx_rb_ref(i)=diff(x_rb_ref(i), t); dy_rb_ref(i)=diff(y_rb_ref(i), t);
    theta_rb_ref(i)=atan2(dy_rb_ref(i), dx_rb_ref(i));

    v_rb_ref(i)=sqrt(dx_rb_ref(i)^2+dy_rb_ref(i)^2);
    omega_rb_ref(i)=diff(theta_rb_ref(i), t);
end

% Generate functions from symbolic expressions (for computational speed)
matlabFunction(x_ob_ref, 'File', 'x_ob_ref.m');
matlabFunction(y_ob_ref, 'File', 'y_ob_ref.m');
matlabFunction(theta_ob_ref, 'File', 'theta_ob_ref.m');

matlabFunction(x_rb_ref, 'File', 'x_rb_ref.m');
matlabFunction(y_rb_ref, 'File', 'y_rb_ref.m');
matlabFunction(theta_rb_ref, 'File', 'theta_rb_ref.m');

matlabFunction(v_rb_ref, 'File', 'v_rb_ref.m');
matlabFunction(omega_rb_ref, 'File', 'omega_rb_ref.m');

clearvars x_ob_ref y_ob_ref theta_ob_ref x_rb_ref y_rb_ref theta_rb_ref v_rb_ref omega_rb_ref

if VIEW_TRAJ
    t_arr=linspace(0, T_END, 1000);

    x_ob_ref_arr=zeros(1, length(t_arr));
    y_ob_ref_arr=zeros(1, length(t_arr));
    theta_ob_ref_arr=zeros(1, length(t_arr));

    x_rb_ref_arr=zeros(2, length(t_arr));
    y_rb_ref_arr=zeros(2, length(t_arr));
    theta_rb_ref_arr=zeros(2, length(t_arr));

    v_rb_ref_arr=zeros(2, length(t_arr));
    omega_rb_ref_arr=zeros(2, length(t_arr));

    for i=1:length(t_arr)
        x_ob_ref_arr(i)=x_ob_ref(t_arr(i));
        y_ob_ref_arr(i)=y_ob_ref(t_arr(i));
        theta_ob_ref_arr(i)=theta_ob_ref(t_arr(i));

        x_rb_ref_arr(:, i)=x_rb_ref(t_arr(i));
        y_rb_ref_arr(:, i)=y_rb_ref(t_arr(i));
        theta_rb_ref_arr(:, i)=theta_rb_ref(t_arr(i));
        %theta_rb_ref_arr(:, i)=0;

        v_rb_ref_arr(:, i)=v_rb_ref(t_arr(i));
        omega_rb_ref_arr(:, i)=omega_rb_ref(t_arr(i));
    end

    robot_colors=['b', 'g', 'c', 'm'];

    figure("Name", "Trajectory preview");
    subplot(4, ROBOT_NUM, 1:3*ROBOT_NUM)
    hold on
    plot(x_ob_ref_arr, y_ob_ref_arr, '-r', 'LineWidth', 1.5, 'DisplayName', 'Object')
    for i=1:ROBOT_NUM
        plot(x_rb_ref_arr(i, :), y_rb_ref_arr(i, :),...
            'Color', robot_colors(i), 'LineWidth', 1.5, 'DisplayName', sprintf('Robot %d', i))
    end
    plot(polyshape([-5, -5, +5, +5], [-4, +3, +3, -4]), 'FaceColor', 'none', 'LineWidth', 3, 'DisplayName', 'Boundary');
    hold off
    ylim([-4.5, +3.5]); xlabel('x [m]'); ylabel('y [m]');
    legend('Location', 'northeast')
    axis equal; grid on;
    title("Reference Object and Robot Positions")

    for i=1:ROBOT_NUM
        subplot(4, ROBOT_NUM, 3*ROBOT_NUM+i)
        hold on
        plot(t_arr, rad2deg(unwrap(theta_rb_ref_arr(i, :))),...
            'Color', robot_colors(i), 'LineWidth', 1.5, 'DisplayName', sprintf('Robot %d', i))
        plot(t_arr, rad2deg(unwrap(theta_ob_ref_arr)), ':r', 'LineWidth', 1.5, 'DisplayName', 'Object')
        xlabel('x [m]'); ylabel('\theta [deg]'); legend('Location', 'northeast');
        title(sprintf("Yaw Angles, Robot %d", i));
        if traj_type==1
            ylim([-180, +180])
        end
    end

    drawnow;

    figure("Name", "Robot commands")
    for i=1:ROBOT_NUM
        subplot(2, ROBOT_NUM, 2*i-1)
        plot(t_arr, v_rb_ref_arr(i, :), '-r')
        xlabel('t [s]'); ylabel('v_{ref} (FROM PLANNER) [m/s]')
        title(sprintf("Reference Lin. Vel., Robot %d (FROM PLANNER)", i));
        if traj_type==1 || traj_type==3
            ylim([-2.0*CHAR_V1, +2.0*CHAR_V1])
        end

        subplot(2, ROBOT_NUM, 2*i)
        plot(t_arr, omega_rb_ref_arr(i, :), '-b')
        xlabel('t [s]'); ylabel('\omega_{ref} (FROM PLANNER) [rad/s]')
        title(sprintf("Reference Ang. Vel., Robot %d (FROM PLANNER)", i));
        if traj_type==1 || traj_type==3
            ylim([-2.0*CHAR_OMEGA1, +2.0*CHAR_OMEGA1])
        end
    end
end

%% 3. Publish Reference Positoins and Velocities

% Generate node
node=ros2node('/matlab_testPlanner_node', ID);

% Generate publishers
pth_publisher=cell(1, ROBOT_NUM);   % Path publisher
pth_message=cell(1, ROBOT_NUM);     % Path message

% ===== FOR TESTING, DELETE IN EXPERIMENTS =====
cnt_publisher=cell(1, ROBOT_NUM);
cnt_message=cell(1, ROBOT_NUM);
% ==============================================

for i=1:ROBOT_NUM
    % TODO: Topic name and type for "planner -> ground station"
    pth_publisher{i}=ros2publisher(node, sprintf('/robot_%d/planned_path', i), 'nav_msgs/Path');
    
    pth_message{i}=ros2message(pth_publisher{i});
    
    % ===== FOR TESTING, DELETE IN EXPERIMENTS =====
    cnt_publisher{i}=ros2publisher(node, sprintf('/robot%d/cmd_vel', i), 'geometry_msgs/TwistStamped');
    cnt_message{i}=ros2message(cnt_publisher{i});
    % ==============================================
end

cmd_publisher=ros2publisher(node, sprintf('/cmd_vel_list'), 'std_msgs/Float64MultiArray');
cmd_message=ros2message(cmd_publisher);


fprintf("\n")
publish_true=input("Start publishing planner outputs? (Press 'y' after visualizer is running)...[y/n] ", 's');

if ~strcmp(publish_true, 'y')
    return
end

t_srt=double(ros2time(node, "now").sec)+(1e-9)*double(ros2time(node, "now").nanosec);   % Time of first publish in Unix timestamp
t_now=t_srt;
while true
    t_now=double(ros2time(node, "now").sec)+(1e-9)*double(ros2time(node, "now").nanosec);

    temp_x=zeros(ROBOT_NUM, N_CMD_LIST);
    temp_y=zeros(ROBOT_NUM, N_CMD_LIST);
    temp_theta=zeros(ROBOT_NUM, N_CMD_LIST);

    temp_t=zeros(1, N_CMD_LIST);
    temp_v=zeros(ROBOT_NUM, N_CMD_LIST);
    temp_omega=zeros(ROBOT_NUM, N_CMD_LIST);
    for j=1:N_CMD_LIST
        temp_t(j)=t_now+j*DT;
        temp_v(:, j)=v_rb_ref(mod(temp_t(j)-t_srt, T_END));
        temp_omega(:, j)=omega_rb_ref(mod(temp_t(j)-t_srt, T_END));

        temp_x(:, j)=x_rb_ref(mod(temp_t(j)-t_srt, T_END));
        temp_y(:, j)=y_rb_ref(mod(temp_t(j)-t_srt, T_END));
        temp_theta(:, j)=theta_rb_ref(mod(temp_t(j)-t_srt, T_END));
    end

    for i=1:ROBOT_NUM
        temp_poses=repmat(ros2message('geometry_msgs/PoseStamped'), 1, N_CMD_LIST);
        for j=1:N_CMD_LIST
            %message{i}.poses.header.stamp.sec=t_now;     % Publish stamped time with Unix timestamp
            temp_poses(j).pose.position.x=temp_x(i, j);
            temp_poses(j).pose.position.y=temp_y(i, j);
            temp_poses(j).pose.position.z=0;

            quat=eul2quat([temp_theta(i, j), 0, 0], "ZYX");
            temp_poses(j).pose.orientation.w=quat(1);
            temp_poses(j).pose.orientation.x=quat(2);
            temp_poses(j).pose.orientation.y=quat(3);
            temp_poses(j).pose.orientation.z=quat(4);

            temp_poses(j).stamp.sec=floor(t_arr(j));
            temp_poses(j).stamp.nanosec=(t_arr(j)-floor(t_arr(j)))*(1e+9);
        end
        pth_message{i}.poses=temp_poses;
        pth_message{i}.header.frame_id='world';
        send(pth_publisher{i}, pth_message{i});

        % ===== FOR TESTING, DELETE IN EXPERIMENTS =====
        cnt_message{i}.twist.linear.x=temp_v(i, 1);
        cnt_message{i}.twist.angular.z=temp_omega(i, 1);
        %send(cnt_publisher{i}, cnt_message{i});
        % ==============================================
    end
    cmd_message.data=zeros(3*N_CMD_LIST*ROBOT_NUM, 1);
    for j=1:N_CMD_LIST
        for i=1:ROBOT_NUM
            cmd_message.data(3*ROBOT_NUM*(j-1)+3*(i-1)+1:3*ROBOT_NUM*(j-1)+3*(i-1)+3, 1)...
                =[temp_t(j); temp_v(i, j); temp_omega(i, j)];
        end
    end
    cmd_message.layout.dim.size=uint32(ROBOT_NUM*N_CMD_LIST);
    send(cmd_publisher, cmd_message);

    fprintf("========== PUBLISH REPORT ==========\n")
    for i=1:ROBOT_NUM
        fprintf("Robot %d \n", i)
        fprintf("Time of publish                          = %.6f \n", t_now-t_srt);
        fprintf("pth_message{%d}.poses.pose.position.x    = %.6f, %.6f ... \n",...
            i, pth_message{i}.poses(1).pose.position.x, pth_message{i}.poses(2).pose.position.x);
        fprintf("pth_message{%d}.poses.pose.position.y    = %.6f, %.6f ... \n",...
            i, pth_message{i}.poses(1).pose.position.y, pth_message{i}.poses(2).pose.position.y);
        fprintf("pth_message{%d}.poses.pose.orientation.z = %.6f, %.6f ... \n",...
            i, pth_message{i}.poses(1).pose.orientation.z, pth_message{i}.poses(2).pose.orientation.z);
        fprintf("cmd_message{%d}, Row 1, Column 1 (time)  = %.6f \n", i, cmd_message.data(ROBOT_NUM*(i-1)+1, 1));
        fprintf("cmd_message{%d}, Row 2, Column 1 (v)     = %.6f \n", i, cmd_message.data(ROBOT_NUM*(i-1)+2, 1));
        fprintf("cmd_message{%d}, Row 3, Column 1 (omega) = %.6f \n", i, cmd_message.data(ROBOT_NUM*(i-1)+3, 1));
        fprintf("\n")
    end
    fprintf("====================================\n\n")

end

clear node pth_publiser cmd_publisher
