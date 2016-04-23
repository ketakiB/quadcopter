%****************************************************
% Kalman Filter
%****************************************************

R_k = 1e-3*diag([2.5e-5*ones(1,3), 7.57e-5*ones(1,3)]); % Covariance matrix measurement noise
Q_k  = 1e-5*diag([1*ones(1,3),1e3*ones(1,3),1*ones(1,6)]);   % Covariance matrix process noise
%Q_k = 1e-5*diag([1*ones(1,3),1e3*ones(1,2),1e5,1*ones(1,6)]); % different Q for when payload on z
G = eye(nx);        % Matrix that is multiplied with process noise in state equation

% Compute Gain matrix
[M,~,~,~] = dlqe(A_d,G,C_d,Q_k,R_k);
L = A_d*M;

% State space model for the Kalman Filter
A_k = A_d-L*C_d;
B_k = [B_d-L*D_d, L];
% output of the kalman filter is just the estimated states
C_k = eye(nx); 
D_k = zeros(nx,nu+ny);