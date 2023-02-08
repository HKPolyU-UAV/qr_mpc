function model = quadrotor_gazebo()

import casadi.*

%% system dimensions
nx = 8;
nu = 3;

%% system parameters

g = 9.81;
hover_thrust = 0.7;
tau_phi = 0.1667;                               % Inner-loop controller time constants
tau_theta = 0.1667;

%% states

x = SX.sym('x');                    % earth position x
y = SX.sym('y');                    % earth position y
z = SX.sym('z');                    % earth position z
u = SX.sym('u');                    % earth velocity x
v = SX.sym('v');                    % earth velocity y
w = SX.sym('w');                    % earth velocity z
phi = SX.sym('phi');                % roll angle phi
theta = SX.sym('theta');            % pitch angle
sym_x = vertcat(x,y,z,u,v,w,phi,theta);

%% controls

thrust = SX.sym('thrust');          % thrust command
phi_cmd = SX.sym('phi_cmd');        % roll angle command
theta_cmd = SX.sym('theta_cmd');    % pitch angle command
sym_u = vertcat(thrust,phi_cmd,theta_cmd);

%% xdot for f_impl

x_dot = SX.sym('x_dot');
y_dot = SX.sym('y_dot');
z_dot = SX.sym('z_dot');
u_dot = SX.sym('u_dot');
v_dot = SX.sym('v_dot');
w_dot = SX.sym('w_dot');
phi_dot = SX.sym('phi_dot');
theta_dot = SX.sym('theta_dot');
sym_xdot = vertcat(x_dot,y_dot,z_dot,u_dot,v_dot,w_dot,phi_dot,theta_dot);

%% dynamics

dx = u;
dy = v;
dz = w;
du = sin(theta) * cos(phi) * thrust/hover_thrust*g;
dv = -sin(phi) * thrust/hover_thrust*g;
dw = -g + cos(theta) * cos(phi) * thrust/hover_thrust*g;
dphi = (phi_cmd - phi) / tau_phi;
dtheta = (theta_cmd - theta) / tau_theta;

expr_f_expl = vertcat(dx,dy,dz,du,dv,dw,dphi,dtheta);
expr_f_impl = expr_f_expl - sym_xdot;

%% constraints

expr_h = sym_u;

%% cost

W_x = diag([60 60 60 10 10 10 10 10]);
W_u = diag([3000 300 300]);

expr_ext_cost_e = sym_x'* W_x * sym_x;
expr_ext_cost = expr_ext_cost_e + sym_u' * W_u * sym_u;
% nonlinear least sqares
cost_expr_y = vertcat(sym_x, sym_u);
W = blkdiag(W_x, W_u);
model.cost_expr_y_e = sym_x;
model.W_e = W_x;

%% populate structure

model.nx = nx;
model.nu = nu;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
model.expr_h = expr_h;
model.expr_ext_cost = expr_ext_cost;
model.expr_ext_cost_e = expr_ext_cost_e;

model.cost_expr_y = cost_expr_y;
model.W = W;

end