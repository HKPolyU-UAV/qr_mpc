%% Generate reference trajectory for NMPC

clear all;
clc

%% Parameters

sample_time = 0.05;     % seconds
duration = 60;          % seconds
hover_thrust = 0.7;
r = 1.5;
v = 5;

x0 = 0.5;
y0 = 0.5;
z0 = 1;

%% Trajectory

traj = zeros(duration/sample_time+1,11);    % x y z u v w phi theta thrust phi_cmd theta_cmd
t = 0:sample_time:duration;

traj(:,1) = -r*cos(t*v/r)+x0;
traj(:,2) = -r*sin(t*v/r)+y0;
traj(:,3) = z0;
traj(:,4) = v*sin(t*v/r);
traj(:,5) = -v*cos(t*v/r);
traj(:,6) = 0;
traj(:,7) = 0;
traj(:,8) = 0;
traj(:,9) = hover_thrust;
traj(:,10) = 0;
traj(:,11) = 0;

%% Write to txt

fid = fopen('circle.txt','w');
fprintf(fid,'%f %f %f %f %f %f %f %f %f %f %f \n',traj');
fclose(fid);