function J = vehicule_model(x, M1, M2, ident_data)
    
    dt = ident_data.dt;
    Ndata = numel(ident_data.w);
 
    M1 = M1.*reshape(x(1:16)' ,4,4);
    M2 = M2.*reshape(x(17:32)',4,4);
    
    test_ss = ss(-M2, M1, eye(4), zeros(4));
    test_ss = c2d(test_ss, dt);
    
    J = 0;
    for dataid = 1:Ndata
        U = ident_data.v{dataid};
        T = 0:dt:length(ident_data.v{dataid})*dt-dt;
        X0 = ident_data.w{dataid}(1,:);
        w_model = lsim(test_ss, U, T, X0);
        J = J + sum((ident_data.w{dataid}(:) - w_model(:)).^2);
    end

end