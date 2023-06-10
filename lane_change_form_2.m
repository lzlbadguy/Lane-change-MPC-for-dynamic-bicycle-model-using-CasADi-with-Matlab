clc;
clear;
%% Constant
lf=1.2;
lr=1.6;
%% Casadi define
import casadi.*
Ts_p = 0.2;         % sampling time inside the MPC
N = 15;             % length of the prediction horizon
n_states = 6;       % number of states
n_controls = 2;     % number of inputs

P = SX.sym('P',n_states + 2);   % 2 reference point [v^{r}_{x},Y^{r}] over a prediction horizon
U = SX.sym('U',n_controls,N);   % 2 control [ax;delta_{f}] over a prediction horizon
X = SX.sym('X',n_states,(N+1)); % 6 states [vx;vy;psi_dot;psi;X;Y] over a prediction horizon

X(:,1) = P(1:6);
for k = 1:N
    X(:,k+1) = vehicle_dynamic_model_6_states(X(:,k),U(:,k),Ts_p); % update the prediction model
end

obj = 0; % Objective function
g = [];  % constraints vector

Q=1000;
R=1;

% compute objective
for k=1:N
    obj = obj+0.9^(k-1)*(X(1,k+1)-P(7))'*Q*(X(1,k+1)-P(7))/25^2;
    obj = obj+0.9^(k-1)*(X(6,k+1)-P(8))'*Q*(X(6,k+1)-P(8))/3.5^2;
    obj = obj+(U(1,k))'*R*(U(1,k))/(2*sqrt(2))^2;
    obj = obj+(U(2,k))'*R*(U(2,k))/(pi/6)^2;
    if k>1
        obj = obj+(U(1,k)-U(1,k-1))'*R*(U(1,k)-U(1,k-1))/(1.5)^2;
        obj = obj+(U(2,k)-U(2,k-1))'*R*(U(2,k)-U(2,k-1))/(pi/12)^2;
    end
end

% compute constraints
for k = 1:N+1   % box constraints due to the map margins
    g = [g ; X(1,k)];   % constraints vector for state vx
    g = [g ; X(2,k)];   % constraints vector for state vy
    g = [g ; X(6,k)];   % constraints vector for state Y
    if k>1 && k<N+1
        g = [g ; (U(1,k)-U(1,k-1))]; % constraints vector for the changing rate of ax
        g = [g ; (U(2,k)-U(2,k-1))]; % constraints vector for the changing rate of delta_f
    end
end

% make the decision variables one column vector
OPT_variables = reshape(U,2*N,1);
nlp_prob = struct('f', obj, 'x', OPT_variables, 'g', g, 'p', P);

opts = struct;
opts.ipopt.max_iter = 100;
opts.ipopt.print_level =0;%0,3
opts.print_time = 0;
opts.ipopt.acceptable_tol =1e-8;
opts.ipopt.acceptable_obj_change_tol = 1e-6;

solver = nlpsol('solver', 'ipopt', nlp_prob,opts);

args = struct;
ubg=[]; % upper constraints container for the vehicle states and some
lbg=[]; % lower constraints container for the vehicle states and some

for k=1:N+1
    lbg=[lbg;0];    % lower constraints for vx
    lbg=[lbg;-5];   % lower constraints for vy
    lbg=[lbg;-2];   % lower constraints for Y
    ubg=[ubg;30];   % upper constraints for vx
    ubg=[ubg;5];    % upper constraints for vy
    ubg=[ubg;2];    % upper constraints for Y
        if k>1 && k<N+1
            lbg=[lbg;-3*Ts_p];      % lower constraints for changing rate of ax
            lbg=[lbg;-pi/36*Ts_p];  % lower constraints for changing rate of delta_{f}
            ubg=[ubg;1.5*Ts_p];     % upper onstraints for changing rate of ax
            ubg=[ubg;pi/36*Ts_p];   % upper onstraints for changing rate of delta_{f}
        end
end
args.ubg=ubg;
args.lbg=lbg;

ubx=[]; % upper constraints container
lbx=[]; % lower constraints container

for k=1:N
   ubx=[ubx;2*sqrt(2)];     % upper constraints for ax
   ubx=[ubx;pi/18];         % upper constraints for delta_{f}
   lbx=[lbx;-2*sqrt(2)];    % lower constraints for ax
   lbx=[lbx;-pi/18];        % lower constraints for delta_{f}
end

args.ubx=ubx;
args.lbx=lbx;

%% THE SIMULATION LOOP SHOULD START FROM HERE
t = 0;
x = [20; 0; 0; 0; 0; 1.75];    % initial condition.
xs = [30; -1.75]; % Reference posture.
u=[0;0];

th=[];
xh=[];
uh=[];
th=[th,t];
xh=[xh,x];
uh=[uh,u];
simtime=15;
Ts_sim=0.1;
u0 = zeros(N,2);

% Define the standard deviation for the measurement noise.
sigma=eye(6);
sigma(1,1)=0.1;            % the standard deviation for the measurement noise of vx=0.1
sigma(2,2)=0;
sigma(3,3)=0;
sigma(4,4)=0;
sigma(5,5)=0;
sigma(6,6)=0.01;           % the standard deviation for the measurement noise of Y=0.01
noise=sigma*randn(6,1);     % Generate the noise based on the standard deviation
x_noise=x+noise;           % Generate noisy system states measurement

for i=1:(simtime/Ts_sim)
    tic;
    if i>(simtime/Ts_sim)/2
        xs = [20; 1.75];
    end
    args.p   = [x;xs];
    args.x0 = reshape(u0',2*N,1);
    sol = solver('x0', args.x0, 'lbx', args.lbx, 'ubx', args.ubx,...
            'lbg', args.lbg, 'ubg', args.ubg,'p',args.p);   
    uout = reshape(full(sol.x)',2,N)';
    ulast=u;
    u=reshape(uout(1,:),2,1);
    x = vehicle_dynamic_model_6_states(x,u,Ts_sim);   

    t=t+Ts_sim;
           
    u0 = [uout(2:N,:);uout(N,:)];
    th=[th,t];
    xh=[xh,x];
    uh=[uh,u];
    sample_time(i)=toc;
end
Simulation_run_time=sum(sample_time)
Generate_plot_CasAdi;