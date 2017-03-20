function J = vehicule_model_ident(x, M1, M2, ident_data)
    
    dt = ident_data.dt;
    Ndata = numel(ident_data.w);
 
    M1 = M1*reshape(x(1:16)' ,4,4);
    M2 = M2*reshape(x(17:32)',4,4);
    
    J = 0;
    for dataid = 1:Ndata
        w_model = zeros(size(ident_data.w{dataid}));
        v = zeros(size(ident_data.v{dataid}));
        w_model(1) = ident_data.w{dataid}(1);
        for k = 2:length(w_model)
            alpha = M1*v(k-1,:)' - M2*w_model(k-1,:)';
            w_model(k,:) = w_model(k-1,:)' + dt*alpha;
        end
        J = J + sum((ident_data.w{dataid}(:) - w_model(:)).^2);
    end
end