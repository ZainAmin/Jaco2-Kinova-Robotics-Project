% Introduction to Robotics, Semester 2
% Erasmus Mundus Joint Master Degree in Medical Imaging and Applications [2022 - 2024]
% Semester Project by MUHAMMAD ZAIN AMIN

% Part 3

clear         
clc        
close all 

% ==================== Denavit-Hartenberg Parameters ==================== %

a = zeros(7, 1, 'like', zeros(1));

alpha = [repmat(pi/2, 6, 1); 0]; 

d = [0.2755 0 -0.410 -0.0098 -0.3111 0 0.2638]';

theta = [pi; zeros(2, 1); pi/4; zeros(3, 1)];

% ======================== Trajectory Parameters ======================== %

% The trajectory duration is 2 seconds
initial_time = 0;               
final_time = 2;

% Set the sampling time
T  = 0.005;

% Total time duration for trajectory
t  = linspace(initial_time, final_time, (final_time - initial_time) / T + 1);

% Total number of samples
N  = length(t);

% Trajectory initial and final values
traj_initial_value = 0;                 
traj_final_value = 1;

% Cruise Velocity
dsc = 0.60; 

% ====================== Trajectory Null Matrices ======================= %

% Joints values, 7 degrees of freedom
q = zeros(7, N); 

% Joints velocities
dq = zeros(7, N);

% Assigned the joints values
q(:,1) = theta;

% Current end-effector position
pos = repmat(zeros(3, 1), 1, N);

% Desired position tajectory
posd = repmat(zeros(3, 1), 1, N);

% Desired linear velocity
dposd = repmat(zeros(3, 1), 1, N);

% Position error
error_pos = repmat(zeros(3, 1), 1, N);

% Current end-effector orientation
quat = repmat(zeros(4, 1), 1, N);

% Desired position
quatd = repmat(zeros(4, 1), 1, N);

% Desired angular velocity
wd = repmat(zeros(3, 1), 1, N);

% Orientation error
error_or = repmat(zeros(3, 1), 1, N);

% ============================== DH TABLE =============================== %

% Constructed the DH Table
DH = [a(:), alpha(:), d(:), theta(:)];

% ========================= Direct Kinematics =========================== %

% Feed the DH table 
T0 = DirectKinematics(DH);

% Initial end-effector position taken from the Direct Kinematics
pos(:,1)  = T0(1:3,4,end);

% Initial orientation
quat(:, 1) = Rot2Quat(T0(1:3, 1:3, end)); 

% Diagonal matrix for position error
kpos = 10 * eye(3);

% Diagonal matrix for orientation error
kor = 10 * eye(3);

% Final end-effector position
posf = repmat([0.2 0.2 0.2 0.2 0.2], 3, 1);
posf(2, 2:3) = 0.4;
posf(3, 3:4) = 0.4;

% =================== Nested Loop for the Robot Trajectory ===================== %

for j = 1:5

    for i = 2:N

        r_iter = 50;
        [si, dsi, ddsi] = trapezoidal(traj_initial_value, traj_final_value, dsc, final_time, t(i-1));
        
        % Desired position of the end-effector
        posd(:, i-1) = pos(:, 1) + si * ( posf(:,j) - pos(:, 1));

        % Desired linear velocity
        dposd(:, i-1) = dsi * ( posf(:,j) - pos(:, 1));
        
        % Desired orientation
        quatd(:, i-1) = quat(:, 1);

        % Desired angular velocity
        wd(:, i-1) = [0; 0; 0]; 
        
        % Direct Kinematics calculation
        DH = [a, alpha, d, q(:, i-1)];
        Tim1 = DirectKinematics(DH);

        % Current position
        pos(:,i-1) = Tim1(1:3,4,end);

        % Current orientation
        quat(:, i-1) = Rot2Quat(Tim1(1:3, 1:3, end));

        % Position error
        error_pos(:, i-1) = posd(:, i-1) - pos(:, i-1);

        % Orientation error
        error_or(:, i-1) = QuatError(quatd(:, i-1), quat(:, i-1));

        % Combination of Linear velocity and Angular velocity
        vd = [dposd(:, i-1); wd(:, i-1)];

        % Jacobian Matrix
        J = Jacobian(DH);

        % Put the joints velocities
        dq(:, i-1) = pinv(J)*(vd + [kpos*error_pos(:, i-1); kor*error_or(:, i-1)]);

        % Put the joints values
        q(:, i) = q(:, i-1) + dq(:, i-1)*T;

        if(rem(i,r_iter) == 0)
            DrawRobot(DH)
            pause(0)
        end

    end

% ==================== Trajectory of the Last Step ====================== %

    % Direct Kinematics calculation
    DH = [a, alpha, d, q(:, N)];
    TN = DirectKinematics(DH);
    robot_stops_time = 0.5;

    % Current position
    pos(:, N) = TN(1:3, 4, end);

    % Current orientation
    quat(:, N) = Rot2Quat(TN(1:3, 1:3, end));

    % Position error
    error_pos(:, i-1) = posf(:,j)-pos(:, N);

    % Orientation error
    error_or(:, N) = QuatError(quatd(:, N), quat(:, 1));

    % Robot pause time at each corner
    pause(robot_stops_time)

    quat(:,1) = quat(:, N);
    q(:,1) = q(:, N);
    pos(:, 1) = pos(:, N);

end

% ============================== Results ================================ %

dq = diff(q, 1, 2)/T;

% Plot the joint positions
figure('Name', 'Joint Positions')

subplot(7, 1, 1)
plot(t, q(1, :))
xlabel('time [s]')
ylabel('Joint %d position [rad]')

subplot(7, 1, 2)
plot(t, q(2, :))
xlabel('time [s]')
ylabel('Joint %d position [rad]')

subplot(7, 1, 3)
plot(t, q(3, :))
xlabel('time [s]')
ylabel('Joint %d position [rad]')

subplot(7, 1, 4)
plot(t, q(4, :))
xlabel('time [s]')
ylabel('Joint %d position [rad]')

subplot(7, 1, 5)
plot(t, q(5, :))
xlabel('time [s]')
ylabel('Joint %d position [rad]')

subplot(7, 1, 6)
plot(t, q(6, :))
xlabel('time [s]')
ylabel('Joint %d position [rad]')

subplot(7, 1, 7)
plot(t, q(7, :))
xlabel('time [s]')
ylabel('Joint %d position [rad]')

% Plot the joint velocities
figure('Name', 'Joint Velocities')
subplot(7, 1, 1)
plot(t(1:end-1), dq(1, :))
xlabel('Time [s]')
ylabel('Joint %d velocity [rad/s]')

subplot(7, 1, 2)
plot(t(1:end-1), dq(2, :))
xlabel('Time [s]')
ylabel('Joint %d velocity [rad/s]')

subplot(7, 1, 3)
plot(t(1:end-1), dq(3, :))
xlabel('Time [s]')
ylabel('Joint %d velocity [rad/s]')

subplot(7, 1, 4)
plot(t(1:end-1), dq(4, :))
xlabel('Time [s]')
ylabel('Joint %d velocity [rad/s]')

subplot(7, 1, 5)
plot(t(1:end-1), dq(5, :))
xlabel('Time [s]')
ylabel('Joint %d velocity [rad/s]')

subplot(7, 1, 6)
plot(t(1:end-1), dq(6, :))
xlabel('Time [s]')
ylabel('Joint %d velocity [rad/s]')

subplot(7, 1, 7)
plot(t(1:end-1), dq(7, :))
xlabel('Time [s]')
ylabel('Joint %d velocity [rad/s]')


% Plot the position errors
figure('Name', 'Position Errors');

% (X-axis)
subplot(3, 1, 1);
plot(t, error_pos(1, :));   
xlabel('time [s]');
ylabel('epos_x [m]');

% (Y-axis)
subplot(3, 1, 2);
plot(t, error_pos(2, :));   
xlabel('time [s]');
ylabel('epos_y [m]');

% (Z-axis)
subplot(3, 1, 3);
plot(t, error_pos(3, :));   
xlabel('time [s]');
ylabel('epos_z [m]');

% Plot the Orientation errors
figure('Name', 'Orientation Errors');

% (X-axis)
subplot(3, 1, 1);
plot(t, error_or(1, :));   
xlabel('time [s]');
ylabel('epos_x [m]');

% (Y-axis)
subplot(3, 1, 2);
plot(t, error_or(2, :));   
xlabel('time [s]');
ylabel('epos_y [m]');

% (Z-axis)
subplot(3, 1, 3);
plot(t, error_or(3, :));   
xlabel('time [s]');
ylabel('epos_z [m]');

% ================================ Ends ================================= %