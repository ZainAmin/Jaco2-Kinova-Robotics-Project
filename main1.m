% Introduction to Robotics, Semester 2
% Erasmus Mundus Joint Master Degree in Medical Imaging and Applications [2022 - 2024]
% Semester Project by MUHAMMAD ZAIN AMIN

% Part 1

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

% Set the sampling time to 0.1 ms
T  = 0.001;

% Total time duration for trajectory
t  = linspace(initial_time, final_time, (final_time - initial_time) / T + 1);

% Total number of samples
N  = length(t);

% Trajectory initial and final values
traj_initial_value = 0;                 
traj_final_value = 2;

% Cruise Velocity
dsc = 1.50;               

% ====================== Trajectory Null Matrices ======================= %

% Joints values, 7 degrees of freedom
q = zeros(7, N); 

% Joints velocities
dq = zeros(7, N);

% Current end-effector position
pos = repmat(zeros(3, 1), 1, N);

% Desired position tajectory
posd = repmat(zeros(3, 1), 1, N);

% Desired linear velocity
dposd = repmat(zeros(3, 1), 1, N);

% Position error
error_pos = repmat(zeros(3, 1), 1, N);

% Assigned the joints values
q(:,1) = theta;

% ============================== DH TABLE =============================== %

% Constructed the DH Table
DH = [a(:), alpha(:), d(:), theta(:)];

% ========================= Direct Kinematics =========================== %

% Feed the DH table 
T0 = DirectKinematics(DH);

% Initial end-effector position taken from the Direct Kinematics
pos(:,1)  = T0(1:3,4,end);

% Final position of the end effector
posf = pos(:,1) - [0; 0; 0.1];

% Diagonal matrix for position error
kpos = diag([10 10 10]); 

% =================== Loop for the Robot Trajectory ===================== %

for i=2:N
    [si, dsi, ddsi] = trapezoidal(traj_initial_value, traj_final_value, dsc, final_time, t(i-1));

    % Desired position of the end-effector
    posd(:,i-1) = pos(:,1) + (posf - pos(:,1))*si;

    % Desired linear velocity
    dposd(:,i-1) = (posf-pos(:,1))*dsi;

    % Direct Kinematics calculation
    DH = [a alpha d q(:, i-1)];
    Tim1 = DirectKinematics(DH);

    % Current position
    pos(:,i-1) = Tim1(1:3,4,end);
    
    % Position error
    error_pos(:, i-1) = posd(:, i-1)-pos(:, i-1); 

    % Linear velocity
    vd = [dposd(:, i-1)];

    % Jacobian Matrix
    J = Jacobian(DH);

    % Put the joints velocities
    dq(:, i-1) = pinv(J(1:3,:)) * (vd + (kpos*error_pos(:,i-1)));

    % Put the joints values
    q(:, i) = q(:, i-1) + dq(:, i-1)*T;

end

% ==================== Trajectory of the Last Step ====================== %

% Direct Kinematics calculation
DH = [a alpha d q(:, N)];
TN = DirectKinematics(DH);

% Current position
pos(:, N) = TN(1:3, 4, end);

% Desired position
posd(:, N) = pos(:,1) + (posf - pos(:,1))*1;

% Position error
error_ps(:, N) = posd(:, N) - pos(:, N);

% ============================== Results ================================ %

% Plot the position errors
figure()

% (X-axis)
subplot(3, 1, 1)
plot(t, error_pos(1, :))   
xlabel('time [s]');
ylabel('epos_x [m]');

% (Y-axis) 
subplot(3, 1, 2)
plot(t, error_pos(2, :))   
xlabel('time [s]');
ylabel('epos_y [m]');

% (Z-axis)
subplot(3, 1, 3)
plot(t, error_pos(3, :))   
xlabel('time [s]');
ylabel('epos_z [m]');

% Display the Jaco Robot initial position
figure();
DH = [a alpha d q(:, 1)];
DrawRobot(DH);

% Display the Jaco Robot final position
DH = [a alpha d q(:, N)];
DrawRobot(DH);

% ================================ Ends ================================= %