%% Generate reference trajectory for NMPC

clear all;
clc

%% Parameters

sample_time = 0.05;     % seconds
duration = 60;          % seconds
hover_thrust = 0.26;

x0 = 0;
y0 = 0;
z0 = 1;

u = 0.1;                % m/s
v = 0;                  % m/s
w = 0;                  % m/s


%% Trajectory

traj = zeros(duration/sample_time+1,11);    % x y z u v w phi theta thrust phi_cmd theta_cmd
t = 0:sample_time:duration;

traj(:,1) = u*t+x0;
traj(:,2) = v*t+y0;
traj(:,3) = w*t+z0;
traj(:,4) = u;
traj(:,5) = v;
traj(:,6) = w;
traj(:,7) = 0;
traj(:,8) = 0;
traj(:,9) = hover_thrust;
traj(:,10) = 0;
traj(:,11) = 0;

%% Write to txt

fid = fopen('test.txt','w');
fprintf(fid,'%f %f %f %f %f %f %f %f %f %f %f \n',traj');
fclose(fid);