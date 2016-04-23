%Constructing the Augmented system with the first 3 outputs only (number of
%states is 12+3 = 15

big_A = [A_d  zeros(nx,3) ;
       -C_d(1:3,:) eye(3)];
big_B = [B_d ; -D_d(1:3,:)];

big_nx = nx+3; % Number of states of the augmented system

% Checking the controlabillity of the Augmented system
disp('Rank of the controllability matrix of the augmented system:');
rank(ctrb(big_A,big_B))