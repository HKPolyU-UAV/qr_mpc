%% Generate reference trajectory for NMPC

clear all;
clc

%% Parameters

sample_time = 0.025;     % seconds
hover_thrust = 0.26;
cycles = 5;
step_interval = 5;

% points_matrix = [-0.5 -0.5 1;...
%           1.5 -0.5 1;...
%           1.5 1.5 1;...
%           -0.5 1.5 1;];
points_matrix = [0 0 1];

%% Trajectory

duration = cycles*size(points_matrix,1)*step_interval;

traj = zeros(duration/sample_time+1,11);    % x y z u v w phi theta thrust phi_cmd theta_cmd
t = 0:sample_time:duration;

traj(:,4) = 0;
traj(:,5) = 0;
traj(:,6) = 0;
traj(:,7) = 0;
traj(:,8) = 0;
traj(:,9) = hover_thrust;
traj(:,10) = 0;
traj(:,11) = 0;

for i = 1:cycles
    for j = 1:size(points_matrix,1)
        traj_start = (i-1)*size(points_matrix,1)*step_interval+(j-1)*step_interval;
        traj_end = (i-1)*size(points_matrix,1)*step_interval+j*step_interval;
        traj(traj_start/sample_time+1:traj_end/sample_time,1:3) = repmat(points_matrix(j,:),step_interval/sample_time,1);
    end
end

traj(end,1:3) = traj(end-1,1:3);
%% Write to txt

fid = fopen('points.txt','w');
fprintf(fid,'%f %f %f %f %f %f %f %f %f %f %f \n',traj');
fclose(fid);