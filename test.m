x = x0_quadcopter;
% opts = odeset('InitialStep',0.01,'RelTol',1e-13,'AbsTol',1e-14);
options = odeset('RelTol',1e-13,'AbsTol',1e-16);
traj = [];
for i=1:size(u_ref,1)
    u = u_ref(i,:)';
    %    [t,y_tmp] = ode113(@(t,y) ffun2(t,y,u) , [0 T_s],x,options);
    
    [~,X]=ode113(@(t,xt)ffun([xt;u+u_eq*ones(nu,1)]),[0,T_s],x,options);
    x = X(end,:)';
    traj = [traj;x];
end

