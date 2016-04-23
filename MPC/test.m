x_cont = x0_quadcopter;
x_disc = x0_quadcopter;
% opts = odeset('InitialStep',0.01,'RelTol',1e-13,'AbsTol',1e-14);
options = odeset('RelTol',1e-13,'AbsTol',1e-16);
traj_cont = [];
traj_disc= [];
for i=1:size(u_ref,1)
    u = u_ref(i,:)';
    %    [t,y_tmp] = ode113(@(t,y) ffun2(t,y,u) , [0 T_s],x,options);
    
    [~,X]=ode113(@(t,xt)ffun([xt;u+u_eq*ones(nu,1)]),[0,T_s],x_cont,options);
    x_cont = X(end,:);
    x_disc = ffun2([x_disc;u+u_eq*ones(nu,1)]);
    traj_cont = [traj_cont; x_cont];
    traj_disc = [traj_disc; x_disc']; 
end

figure
plot3(refs(:,1),refs(:,2),refs(:,3),'ro-');
hold on
plot3(traj_cont(:,1),traj_cont(:,2),traj_cont(:,3));
plot3(traj_disc(:,1),traj_disc(:,2),traj_disc(:,3));
legend({'reference','continuous','discrete'},'FontSize',18);
xlabel('x [m]')
ylabel('y [m]')
zlabel('z [m]')
