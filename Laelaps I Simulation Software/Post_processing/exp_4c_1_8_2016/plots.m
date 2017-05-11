
%% Load data
clear all
load('EXP_4c.mat')

% Size of data (find the vector with the minimum length)
n = min([size(FR_knee_encoder,2),size(FR_hip_encoder,2),...
    size(BR_knee_encoder,2),size(BR_hip_encoder,2),...
    size(FL_knee_encoder,2),size(FL_hip_encoder,2),...
    size(BL_knee_encoder,2),size(BL_hip_encoder,2),...
    size(FR_knee_setpoint,2),size(FR_hip_setpoint,2),...
    size(BR_knee_setpoint,2),size(BR_hip_setpoint,2),...
    size(FL_knee_setpoint,2),size(FL_hip_setpoint,2),...
    size(BL_knee_setpoint,2),size(BL_hip_setpoint,2)]);

%set(gcf,'color','w');
nstart = round(2*n/3);

%% Change time vectors
first_node_value = min([FR_hip_ros_time BR_hip_ros_time ...
    FL_hip_ros_time BL_hip_ros_time FR_knee_ros_time ...
    BR_knee_ros_time FL_knee_ros_time BL_knee_ros_time]);
FR_hip_ros_time = FR_hip_ros_time - first_node_value;
BR_hip_ros_time = BR_hip_ros_time - first_node_value;
FL_hip_ros_time = FL_hip_ros_time - first_node_value;
BL_hip_ros_time = BL_hip_ros_time - first_node_value;
FR_knee_ros_time = FR_knee_ros_time - first_node_value;
BR_knee_ros_time = BR_knee_ros_time - first_node_value;
FL_knee_ros_time = FL_knee_ros_time - first_node_value;
BL_knee_ros_time = BL_knee_ros_time - first_node_value;

%% Convert estimated velocities to velocities after gearbox in RPM

% Encoder and transmission parameters
count_no = 2000;
gear_ratio_hip = 637/12;
gear_ratio_knee = 343/8;
% Hips
FR_hip_angular_speed = FR_hip_angular_speed *2*pi * 9.54929659643 ...
    / (count_no*gear_ratio_hip);
BR_hip_angular_speed = BR_hip_angular_speed *2*pi * 9.54929659643 ...
    / (count_no*gear_ratio_hip);
FL_hip_angular_speed = FL_hip_angular_speed *2*pi * 9.54929659643 ...
    / (count_no*gear_ratio_hip);
BL_hip_angular_speed = BL_hip_angular_speed *2*pi * 9.54929659643 ...
    / (count_no*gear_ratio_hip);
% Knee
FR_knee_angular_speed = FR_knee_angular_speed *2*pi * 9.54929659643 ...
    / (count_no*gear_ratio_knee);
BR_knee_angular_speed = BR_knee_angular_speed *2*pi * 9.54929659643 ...
    / (count_no*gear_ratio_knee);
FL_knee_angular_speed = FL_knee_angular_speed *2*pi * 9.54929659643 ...
    / (count_no*gear_ratio_knee);
BL_knee_angular_speed = BL_knee_angular_speed *2*pi * 9.54929659643 ...
    / (count_no*gear_ratio_knee);

%% Change POD vectors
FR_hip_pid_u = FR_hip_pid_u/10;
BR_hip_pid_u = BR_hip_pid_u/10;
FL_hip_pid_u = FL_hip_pid_u/10;
BL_hip_pid_u = BL_hip_pid_u/10;
FR_knee_pid_u = FR_knee_pid_u/10;
BR_knee_pid_u = BR_knee_pid_u/10;
FL_knee_pid_u = FL_knee_pid_u/10;
BL_knee_pid_u = BL_knee_pid_u/10;



%% Plot encoders and setpoints for hips
figure()
set(gcf, 'Position', [100 50 900 800],'color','w');
subplot(4,1,1)
hold all
plot(FR_hip_ros_time(nstart:n),FR_hip_setpoint(nstart:n),'r','LineWidth',0.1);
plot(FR_hip_ros_time(nstart:n),FR_hip_encoder(nstart:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Counts');
box on
subplot(4,1,2)
hold all
plot(BR_hip_ros_time(nstart:n),BR_hip_setpoint(nstart:n),'r','LineWidth',0.1);
plot(BR_hip_ros_time(nstart:n),BR_hip_encoder(nstart:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Counts');
box on
subplot(4,1,3)
hold all
plot(FL_hip_ros_time(1:n),FL_hip_setpoint(1:n),'r','LineWidth',0.1);
plot(FL_hip_ros_time(1:n),FL_hip_encoder(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Counts');
box on
subplot(4,1,4)
hold all
plot(BL_hip_ros_time(1:n),BL_hip_setpoint(1:n),'r','LineWidth',0.1);
plot(BL_hip_ros_time(1:n),BL_hip_encoder(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Counts');
box on

tightfig;

%% Plot encoders and setpoints for knees
figure()
set(gcf, 'Position', [100 50 900 800],'color','w');
subplot(4,1,1)
hold all
plot(FR_knee_ros_time(1:n),FR_knee_setpoint(1:n),'r','LineWidth',0.1);
plot(FR_knee_ros_time(1:n),FR_knee_encoder(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Counts');
box on
subplot(4,1,2)
hold all
plot(BR_knee_ros_time(1:n),BR_knee_setpoint(1:n),'r','LineWidth',0.1);
plot(BR_knee_ros_time(1:n),BR_knee_encoder(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Counts');
box on
subplot(4,1,3)
hold all
plot(FL_knee_ros_time(1:n),FL_knee_setpoint(1:n),'r','LineWidth',0.1);
plot(FL_knee_ros_time(1:n),FL_knee_encoder(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Counts');
box on
subplot(4,1,4)
hold all
plot(BL_knee_ros_time(1:n),BL_knee_setpoint(1:n),'r','LineWidth',0.1);
plot(BL_knee_ros_time(1:n),BL_knee_encoder(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Counts');
box on

tightfig;

%% Plot estimated angular velocities for hips
figure()
set(gcf, 'Position', [100 50 900 800],'color','w');
subplot(4,1,1)
plot(FR_hip_ros_time(1:n),FR_hip_angular_speed(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Velocity [rpm]');
subplot(4,1,2)
plot(BR_hip_ros_time(1:n),BR_hip_angular_speed(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Velocity [rpm]');
subplot(4,1,3)
plot(FL_hip_ros_time(1:n),FL_hip_angular_speed(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Velocity [rpm]');
subplot(4,1,4)
plot(BL_hip_ros_time(1:n),BL_hip_angular_speed(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Velocity [rpm]');

tightfig;

%% Plot estimated angular velocities for knees
figure()
set(gcf, 'Position', [100 50 900 800],'color','w');
subplot(4,1,1)
plot(FR_knee_ros_time(1:n),FR_knee_angular_speed(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Velocity [rpm]');
subplot(4,1,2)
plot(BR_knee_ros_time(1:n),BR_knee_angular_speed(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Velocity [rpm]');
subplot(4,1,3)
plot(FL_knee_ros_time(1:n),FL_knee_angular_speed(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Velocity [rpm]');
subplot(4,1,4)
plot(BL_knee_ros_time(1:n),BL_knee_angular_speed(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Velocity [rpm]');

tightfig;

%% Plot pid commands for hips
figure()
set(gcf, 'Position', [100 50 900 800],'color','w');
subplot(4,1,1)
plot(FR_hip_ros_time(1:n),FR_hip_pid_u(1:n),'k','LineWidth',0.1);
hold on
plot(FR_hip_ros_time(1:n),35*ones(n,1),'r','LineWidth',0.1);
plot(FR_hip_ros_time(1:n),-35*ones(n,1),'r','LineWidth',0.1);
xlabel('Time [s]');
ylabel('PWM Command [%]');

subplot(4,1,2)
plot(BR_hip_ros_time(1:n),BR_hip_pid_u(1:n),'k','LineWidth',0.1);
hold on
plot(BR_hip_ros_time(1:n),35*ones(n,1),'r','LineWidth',0.1);
plot(BR_hip_ros_time(1:n),-35*ones(n,1),'r','LineWidth',0.1);
xlabel('Time [s]');
ylabel('PWM Command [%]');


subplot(4,1,3)
plot(FL_hip_ros_time(1:n),FL_hip_pid_u(1:n),'k','LineWidth',0.1);
hold on
plot(FL_hip_ros_time(1:n),35*ones(n,1),'r','LineWidth',0.1);
plot(FL_hip_ros_time(1:n),-35*ones(n,1),'r','LineWidth',0.1);
xlabel('Time [s]');
ylabel('PWM Command [%]');

subplot(4,1,4)
plot(BL_hip_ros_time(1:n),BL_hip_pid_u(1:n),'k','LineWidth',0.1);
hold on
plot(BL_hip_ros_time(1:n),35*ones(n,1),'r','LineWidth',0.1);
plot(BL_hip_ros_time(1:n),-35*ones(n,1),'r','LineWidth',0.1);
xlabel('Time [s]');
ylabel('PWM Command [%]');


tightfig;

%% Plot pid commands for knees
figure()
set(gcf, 'Position', [100 50 900 800],'color','w');
subplot(4,1,1)
plot(FR_knee_ros_time(1:n),FR_knee_pid_u(1:n),'k','LineWidth',0.1);
hold on
plot(FR_knee_ros_time(1:n), 35*ones(n,1),'r','LineWidth',0.1)
plot(FR_knee_ros_time(1:n), -35*ones(n,1),'r','LineWidth',0.1)
xlabel('Time [s]');
ylabel('PWM Command [%]');

subplot(4,1,2)
plot(BR_knee_ros_time(1:n),BR_knee_pid_u(1:n),'k','LineWidth',0.1);
hold on
plot(BR_knee_ros_time(1:n), 35*ones(n,1),'r','LineWidth',0.1)
plot(BR_knee_ros_time(1:n), -35*ones(n,1),'r','LineWidth',0.1)
xlabel('Time [s]');
ylabel('PWM Command [%]');

subplot(4,1,3)
plot(FL_knee_ros_time(1:n),FL_knee_pid_u(1:n),'k','LineWidth',0.1);
hold on
plot(FL_knee_ros_time(1:n), 35*ones(n,1),'r','LineWidth',0.1)
plot(FL_knee_ros_time(1:n), -35*ones(n,1),'r','LineWidth',0.1)
xlabel('Time [s]');
ylabel('PWM Command [%]');

subplot(4,1,4)
plot(BL_knee_ros_time(1:n),BL_knee_pid_u(1:n),'k','LineWidth',0.1);
hold on
plot(BL_knee_ros_time(1:n), 35*ones(n,1),'r','LineWidth',0.1)
plot(BL_knee_ros_time(1:n), -35*ones(n,1),'r','LineWidth',0.1)
xlabel('Time [s]');
ylabel('PWM Command [%]');

tightfig;


%% Plot deltat
figure()
set(gcf, 'Position', [100 50 900 800],'color','w');
subplot(4,2,1)
plot(FR_hip_ros_time(1:n), FR_hip_time_delta_t(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Delta t [s]');
subplot(4,2,2)
plot(FL_knee_ros_time(1:n), FR_knee_time_delta_t(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Delta t [s]');
subplot(4,2,3)
plot(BR_hip_ros_time(1:n), BR_hip_time_delta_t(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Delta t [s]');
subplot(4,2,4)
plot(BR_knee_ros_time(1:n), BR_knee_time_delta_t(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Delta t [s]');
subplot(4,2,5)
plot(FL_hip_ros_time(1:n), FL_hip_time_delta_t(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Delta t [s]');
subplot(4,2,6)
plot(FL_knee_ros_time(1:n),FL_knee_time_delta_t(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Delta t [s]');
subplot(4,2,7)
plot(BL_hip_ros_time(1:n), BL_hip_time_delta_t(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Delta t [s]');
subplot(4,2,8)
plot(BL_knee_ros_time(1:n), BL_knee_time_delta_t(1:n),'k','LineWidth',0.1);
xlabel('Time [s]');
ylabel('Delta t [s]');

tightfig;

%% Plot encoders
figure()
set(gcf, 'Position', [100 50 900 800],'color','w');
subplot(2,1,1)
hold all
plot(FR_hip_ros_time(nstart:n),FR_hip_encoder(nstart:n),'LineWidth',0.1);
plot(BR_hip_ros_time(nstart:n),BR_hip_encoder(nstart:n),'LineWidth',0.1);
plot(FL_hip_ros_time(nstart:n),FL_hip_encoder(nstart:n),'LineWidth',0.1);
plot(BL_hip_ros_time(nstart:n),BL_hip_encoder(nstart:n),'LineWidth',0.1);
legend('FR Hip','BR Hip','FL Hip','BL Hip')
xlabel('Time [s]');
ylabel('Counts');
box on

subplot(2,1,2)
hold all
plot(FR_knee_ros_time(nstart:n),FR_knee_encoder(nstart:n),'LineWidth',0.1);
plot(BR_knee_ros_time(nstart:n),BR_knee_encoder(nstart:n),'LineWidth',0.1);
plot(FL_knee_ros_time(nstart:n),FL_knee_encoder(nstart:n),'LineWidth',0.1);
plot(BL_knee_ros_time(nstart:n),BL_knee_encoder(nstart:n),'LineWidth',0.1);
legend('FR Knee','BR Knee','FL Knee','BL Knee')
xlabel('Time [s]');
ylabel('Counts');
box on

tightfig;
