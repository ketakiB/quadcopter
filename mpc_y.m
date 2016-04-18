N = 20; % prediction horizon
umax = 100-u_eq; umin = u_eq; % input limit
zmax = inf; zmin = inf; % room limits 
xymax = inf;
nz=3;
% Fill the reference vector with N repititions of the last input/state such that
% we can also compute the N last inputs with a horizon N
for i=(M+1):(M+N)
    y_ref_vector((i-1)*nz+1:i*nz) = y_ref_vector(end-nz+1:end);
end

G_d = C_d(1:nz,:);
H_d = D_d(1:nz,:);

qdiag = ones(1,nz);
%rdiag = 1e-7*ones(1,nu);
bigQ = repmat(qdiag,1,N);
%bigR = repmat(rdiag,1,N);
H11 = kron(eye(N),G_d'*Q*G_d);
H12 = kron(eye(N),G_d'*Q*H_d);
H21 = kron(eye(N),H_d'*Q*G_d);
H22 = kron(eye(N),H_d'*Q*H_d);
H = [H11, H12; H21, H22];
f0 = 2*[kron(eye(N),G_d'*Q), zeros(nx*N,nz*N); zeros(nu*N,nz*N), kron(eye(N),H_d'*Q)];
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
for k=1:M
    % extract time horizon
    y_ref_now = y_ref_vector((k-1)*nz+1:(k+N-1)*nz);
    % complete optimization matrices
    f = -f0*[y_ref_now; y_ref_now];
    b_eq = zeros(N*nx,1);
    b_eq(1:nx) = A_d*x;
    % solve optimization problem
    xu = quadprog(H,f,A_ineq,b_ineq,A_eq,b_eq);
    %xu = quadprog(H,f,[],[],A_eq,b_eq);
    % take first input to apply 
    u = xu(nx*N+1:nx*N+nu);
    % update state
    y = C_d*x + D_d*u;
    x = A_d*x + B_d*u;
    U_vector(k,:) = u';
    X_vector(k,:) = x';
    Y_vector(k,:) = y';
end

close all


figure
plot(T,Y_vector(:,1:3));
hold on
plot(T,refs(:,1:3),'-.');
legend({'x','y','z','x_{ref}','y_{ref}','z_{ref}'},'FontSize',18);
title('Resulting outputs');
xlabel('T [s]')

figure
plot3(refs(:,1),refs(:,2),refs(:,3),'ro-');
hold on
plot3(Y_vector(:,1),Y_vector(:,2),Y_vector(:,3));
legend({'reference','quadcopter'},'FontSize',18);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
title('Simulation results MPC')

figure
plot(T,X_vector);
legend({'x','y','z','v_x','v_y','v_z','\phi','\tau','\psi','\omega_x','\omega_y','\omega_z'},'FontSize',18);
title('States');
xlabel('T [s]')

figure
plot(T,U_vector);
title('Control inputs');
xlabel('T [s]')




