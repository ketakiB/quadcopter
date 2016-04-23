load 'references.mat';
[M,~]=size(refs);
%M=M/2;

y_ref = [refs(1:M,:), zeros(M,ny-3)];


y_ref_vector = reshape(y_ref',[M*ny,1]);
x0_quadcopter=zeros(nx,1);

N = 50; % prediction horizon 
% NOTE: control inputs are clearly smaller and more regular with larger N
umax = 100-u_eq; umin = u_eq; % input limit
zmax = 6; zmin = 0; % room limits 
xymax = 3;

% Fill the reference vector with N repititions of the last input/state such that
% we can also compute the N last inputs with a horizon N
for i=(M+1):(M+N)
    y_ref_vector((i-1)*ny+1:i*ny) = y_ref_vector(end-ny+1:end);
end

Q = eye(ny); Q(3,3)=1e3;
R = 1e-3*eye(nu);
H11 = kron(eye(N),C_d'*Q*C_d);
H12 = kron(eye(N),C_d'*Q*D_d);
H21 = kron(eye(N),D_d'*Q*C_d);
H22 = kron(eye(N),D_d'*Q*D_d+R);
H = 2*[H11, H12; H21, H22];
f0 = -2*[kron(eye(N),C_d'*Q), zeros(nx*N,ny*N); zeros(nu*N,ny*N), kron(eye(N),D_d'*Q)];
A_eq = [eye(nx*N), zeros(nx*N,nu*N)];
for i=1:N
    if(i<N)
        A_eq(i*nx+1:(i+1)*nx,(i-1)*nx+1:i*nx) = -A_d;
    end
    A_eq((i-1)*nx+1:i*nx, nx*N + (i-1)*nu+1:nx*N + i*nu) = -B_d;
end

% For now, we just limit the space of the quadcopter and the input size
A_ineq = [eye(nx*N), zeros(nx*N,nu*N); -eye(nx*N), zeros(nx*N,nu*N);
       zeros(nu*N,nx*N), eye(nu*N); zeros(nu*N,nx*N), -eye(nu*N)]; 
% inputs between 0 and umax
b_ineq_u_max = repmat([umax],nu,1);
b_ineq_u_min = repmat([umin],nu,1);
% xy coordinates between + and - xymax, z coordinate between 0 and zmax
b_ineq_x_max = [xymax; xymax; zmax; inf*ones(nx-3,1)]; 
b_ineq_x_min = [xymax; xymax; zmin; inf*ones(nx-3,1)];
b_ineq = [repmat(b_ineq_x_max,N,1); repmat(b_ineq_x_min,N,1) ;repmat(b_ineq_u_max,N,1); repmat(b_ineq_u_min,N,1)];

x = x0_quadcopter; 
x_k = x;

U_vector = zeros(M,nu);
X_vector = zeros(M,nx);
X_k_vector = zeros(M,nx);
Y_vector = zeros(M,ny);
%options = odeset('RelTol',1e-13,'AbsTol',1e-16);
for k=1:M
    % extract time horizon
    y_ref_now = y_ref_vector((k-1)*ny+1:(k+N-1)*ny);
    % complete optimization matrices
    f = f0*[y_ref_now; y_ref_now];
    b_eq = zeros(N*nx,1);
    b_eq(1:nx) = A_d*x_k;
    % solve optimization problem
    xu = quadprog(H,f,A_ineq,b_ineq,A_eq,b_eq);
    % xu = quadprog(H,f,[],[],A_eq,b_eq);
    % take first input to apply 
    u = xu(nx*N+1:nx*N+nu);
    % update state
    y = C_d*x;
    [~,X]=ode113(@(t,xt)ffun([xt;u+u_eq*ones(nu,1)]),[0,T_s],x);
    x = X(end,:)';
    x_k = A_k*x_k + B_k*[u; y]; % state estimates
    U_vector(k,:) = u';
    X_vector(k,:) = x';
    X_k_vector(k,:) = x_k';
    Y_vector(k,:) = y';
end
%%
close all

T=T_s*(0:M-1);

figure
plot(T,Y_vector(:,1:3));
hold on
plot(T,y_ref(:,1:3),'--');
legend({'x','y','z','x_{ref}','y_{ref}','z_{ref}'},'FontSize',18);
title('Resulting outputs');
xlabel('T [s]')

figure
plot3(y_ref(:,1),y_ref(:,2),y_ref(:,3),'ro');
hold on
plot3(Y_vector(:,1),Y_vector(:,2),Y_vector(:,3));
legend({'reference','quadcopter'},'FontSize',18);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
title('Simulation results MPC')

figure
plot(T,X_k_vector(:,1:3));
hold on 
plot(T,X_vector(:,1:3),'--');
legend({'x','y','z','x_k','y_k','z_k'},'FontSize',18);
%legend({'x','y','z','v_x','v_y','v_z','\phi','\tau','\psi','\omega_x','\omega_y','\omega_z'},'FontSize',18);
title('States');
xlabel('T [s]')

figure
plot(T,X_vector(:,4:6),'--');
hold on 
plot(T,X_k_vector(:,4:6));
legend({'\phi','\tau','\psi','\phi_k','\tau_k','\psi_k'},'FontSize',18);
%legend({'x','y','z','v_x','v_y','v_z','\phi','\tau','\psi','\omega_x','\omega_y','\omega_z'},'FontSize',18);
title('States');
xlabel('T [s]')

figure
plot(T,U_vector);
title('Control inputs');
xlabel('T [s]')




