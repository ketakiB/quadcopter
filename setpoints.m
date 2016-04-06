load 'ref_trajectory.mat';
%refs = refs(:,1:3);
%refs = refs_new;
[M,~] = size(refs);
% Built optimization problem that finds u such that (y-y_ref) minimized

% refs are organized [x_ref, y_ref, z_ref] but must be flattened in vector
% [x_ref(1); y_ref(1); z_ref(1); ...; y_ref(Tmax); z_ref(Tmax)]

y_ref_vector = reshape(refs',[M*ny,1]);

% construct O = [C ; CA; CA^2; ... ; CA^{Tmax-1}]
% Constructing O
O = zeros(M*ny,nx);
O(1:ny,:) = C_d;
for i=1:M-1
    O(i*ny+1:(i+1)*ny,:) = O((i-1)*ny+1:i*ny,:)*A_d;
end

% construct H = matrix of markov parameters: H_0 = D, H_k = CA^{k-1}B
colH = zeros(M*ny,nu);
colH(1:ny,:)=D_d;
temp = B_d;
for k=1:M-1
    colH(k*ny+1:(k+1)*ny,:)=C_d*temp;
    temp = A_d*temp;
end
H = zeros(M*ny,M*nu);
for i=0:M-1
    H(:,i*nu+1:(i+1)*nu) = colH;
    colH = [zeros(ny, nu); 
            colH(1:end-ny,:)];
end

% Solve optimization problem: find u 
%u_opt = quadprog(H'*H,H'*(O*x0_quadcopter-y_ref_vector),[eye(Tmax*nu);-eye(Tmax*nu)],[100*ones(Tmax*nu,1);zeros(Tmax*nu,1)]);
%u_opt = quadprog(H'*H,H'*(y_ref_vector-O*x0_quadcopter));
Q = diag(repmat([1,1,1,0,0,0],1,M));
R = 1e-7*eye(M*nu);
u_ref_vector = quadprog(R+H'*Q*H,H'*Q'*(y_ref_vector-O*x0_quadcopter));
u_ref_vector=-u_ref_vector;

% reshape the inputs as rowvectors for each time t
u_ref = reshape(u_ref_vector,[nu,M])';

% Simulation to get actual outputs and reference states
[Y, T, x_ref] = lsim(sys,u_ref);

x_ref_vector = reshape(x_ref',[M*nx,1]);

close all

figure,
plot(T,Y);
legend({'x','y','z','\phi','\theta', '\psi'},'FontSize',18);
title('Resulting outputs');
xlabel('T [s]')

figure,
plot(T,refs);
legend({'x','y','z','\phi','\theta', '\psi'},'FontSize',18);
title('Reference outputs');
xlabel('T [s]')

figure,
plot(T,u_ref);
legend({'u_1','u_2','u_3','u_4'},'FontSize',18);
title('Control inputs');
xlabel('T [s]')

figure,
plot(T,x_ref);
legend({'x','y','z','v_x','v_y','v_z','\phi','\theta','\psi','\omega_x','\omega_y','\omega_z'},'FontSize',18);
title('States');
xlabel('T [s]')


figure
plot3(refs(:,1),refs(:,2),refs(:,3),'ro-');
hold on
plot3(Y(:,1),Y(:,2),Y(:,3),'b-');
%plot(Setpoints_output(:,2),Setpoints_output(:,3),'g.-');
legend({'reference','quadcopter'},'FontSize',18);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
title('Simulation results')