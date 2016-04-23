close all

f1 = figure('Position', [1, 500, 1050, 300]);
plot(T,ref_vector,'--');
hold on
plot(T,Y_vector(:,1:3));
legend({'x_{ref}','y_{ref}','z_{ref}','x','y','z'},'FontSize',15,'Location','southwest');
xlabel('T [s]','FontSize',12)
ylabel('[m]','FontSize',12)

f2 = figure();
plot3(Y_vector(:,1),Y_vector(:,2),Y_vector(:,3));
hold on
for k=1:8
    plot3(refs_ss(k,1),refs_ss(k,2),refs_ss(k,3),'ro-');
end
legend({'quadcopter','reference'},'FontSize',15,'Location','best');
xlabel('x [m]','FontSize',12)
ylabel('y [m]','FontSize',12)
zlabel('z [m]','FontSize',12)

f3 = figure('Position', [1, 500, 1050, 300]);
plot(T,X_vector);
legend({'x','y','z','v_x','v_y','v_z','\phi','\tau','\psi','\omega_x','\omega_y','\omega_z'},'FontSize',15,'Orientation','horizontal');
xlabel('T [s]','FontSize',12)

f4 = figure('Position', [1, 500, 1050, 300]);
plot(T,U_vector);
xlabel('T [s]','FontSize',12)
ylabel('u [v^2]','FontSize',12)


f5 = figure('Position', [1, 500, 1050, 300]);
plot(T,X_k_vector(:,1:3));
hold on
plot(T,X_vector(:,1:3),'--');
legend({'x_k','y_k','z_k','x','y','z'},'FontSize',15);
xlabel('T [s]','FontSize',12)
ylabel('[m]','FontSize',12)

f6 = figure('Position', [1, 500, 1050, 300]);
plot(T,X_k_vector(:,4:6));
hold on
plot(T,X_vector(:,4:6),'--');
legend({'\phi_k','\tau_k','\psi_k','\phi','\tau','\psi'},'FontSize',15);
xlabel('T [s]','FontSize',12)
ylabel('[rad]','FontSize',12)
