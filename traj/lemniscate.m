%% Generate reference trajectory for NMPC

clear all;
clc

%% Parameters

sample_time = 0.025;     % seconds
duration = 60;          % seconds
hover_thrust = 0.26;
amp = 2;
frq = 2;

x0 = 0;
y0 = 0;
z0 = 1;

%% Trajectory

traj = zeros(duration/sample_time+1,11);    % x y z u v w phi theta thrust phi_cmd theta_cmd
t = 0:sample_time:duration;

traj(:,1) = amp*cos(t*frq)+x0;
traj(:,2) = amp*sin(t*frq).*cos(t*frq)+y0;
traj(:,3) = z0;
traj(:,4) = -amp*frq*sin(t*frq);
traj(:,5) = amp*frq*cos(t*2*frq);
traj(:,6) = 0;
traj(:,7) = 0;
traj(:,8) = 0;
traj(:,9) = hover_thrust;
traj(:,10) = 0;
traj(:,11) = 0;

%% Write to txt

fid = fopen('lemniscate.txt','w');
fprintf(fid,'%f %f %f %f %f %f %f %f %f %f %f \n',traj');
fclose(fid);