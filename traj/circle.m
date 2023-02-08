%% Generate reference trajectory for NMPC

clear all;
clc

%% Parameters

sample_time = 0.025;     % seconds
duration = 30;          % seconds
hover_thrust = 0.25;
r = 1.5;
v = 4;

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

traj = [repmat(traj(1,:),5/sample_time,1);traj];
traj(1:5/sample_time,4:6) = 0;
traj = [traj;repmat([traj(end,1:3) 0 0 0 0 0 hover_thrust 0 0],5/sample_time,1)];

%% Write to txt

fid = fopen('circle.txt','w');
fprintf(fid,'%f %f %f %f %f %f %f %f %f %f %f \n',traj');
fclose(fid);