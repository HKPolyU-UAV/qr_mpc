clear all

check_acados_requirements()

%% discretization

N = 40;     % prediction horizon steps
T = 1;      % prediction horizon length (seconds)
x0 = [0;0;0;0;0;0;0;0;];
u_min = [0.5 -pi/2 -pi/2];
u_max = [0.9 pi/2 pi/2];

nlp_solver = 'sqp';
qp_solver = 'partial_condensing_hpipm';
qp_solver_cond_N = 5;
sim_method = 'erk';

%% model dynamics

model = quadrotor;
nx = model.nx;
nu = model.nu;
ny = size(model.cost_expr_y, 1);
ny_e = size(model.cost_expr_y_e, 1);

%% model to create the solver

ocp_model = acados_ocp_model();
model_name = 'quadrotor';

%% acados ocp model

ocp_model.set('name',model_name);
ocp_model.set('T',T);

% symbolics
ocp_model.set('sym_x', model.sym_x);
ocp_model.set('sym_u', model.sym_u);
ocp_model.set('sym_xdot',model.sym_xdot);

% cost
ocp_model.set('cost_expr_ext_cost', model.expr_ext_cost);
ocp_model.set('cost_expr_ext_cost_e',model.expr_ext_cost_e);

% dynamics
if (strcmp(sim_method, 'erk'))
    ocp_model.set('dyn_type', 'explicit');
    ocp_model.set('dyn_expr_f', model.expr_f_expl);
else
    ocp_model.set('dyn_type', 'implicit');
    ocp_model.set('dyn_expr_f', model.expr_f_impl);
end

% constraints
ocp_model.set('constr_type', 'auto');
ocp_model.set('constr_expr_h', model.expr_h);
ocp_model.set('constr_lh', u_min);
ocp_model.set('constr_uh', u_max);

ocp_model.set('constr_x0', x0);

%% acados ocp set opts

ocp_opts = acados_ocp_opts();
ocp_opts.set('param_scheme_N',N);
ocp_opts.set('nlp_solver', nlp_solver);
ocp_opts.set('sim_method', sim_method);
ocp_opts.set('qp_solver', qp_solver);
ocp_opts.set('qp_solver_cond_N', qp_solver_cond_N);
ocp_opts.set('ext_fun_compile_flags', '');

%% create ocp solver

ocp = acados_ocp(ocp_model, ocp_opts);

x_traj_init = zeros(nx, N+1);
u_traj_init = zeros(nu,N);

%% call ocp solver

% update initial state
ocp.set('constr_x0', x0);

% set trajectory initialization
ocp.set('init_x', x_traj_init);
ocp.set('init_u', u_traj_init);
ocp.set('init_pi', zeros(nx,N));

ocp.set('constr_lbx', x0, 0);

ocp.solve();
utraj = ocp.get('u');
xtraj = ocp.get('x');

status = ocp.get('status'); % 0 - success
ocp.print('stat')

ocp.generate_c_code;