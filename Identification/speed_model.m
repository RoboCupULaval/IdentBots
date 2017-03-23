function J = speed_model(x, M3, M4, ident_data)

    rotF2V = @(phiFc) [cos(phiFc) sin(phiFc) 0 ; -sin(phiFc) cos(phiFc) 0; 0 0 1];
    dt = ident_data.dt;
    Ndata = numel(ident_data.pvFc);

    M3 = M3.*reshape(x(1:9)', 3, 3);
    M4 = M4.*reshape(x(10:end)', 3, ident_data.nwheel);
    
    J = 0;
    for dataid = 1:Ndata           
        
        pvFc_model = zeros(size(ident_data.pvFc{dataid}));
        rw = ident_data.rw{dataid};
        pvFc_model(ident_data.retard) = ident_data.pvFc{dataid}(ident_data.retard);
        for k = (ident_data.retard+1):length(pvFc_model)
            alpha = M3*pvFc_model(k-1,:)' - M3/rotF2V(ident_data.ppFc{dataid}(k-1, 3))*M4*rw(k-ident_data.retard, :)';
            pvFc_model(k,:) = pvFc_model(k-1,:)' + dt*alpha;
        end
        J = J + sum((ident_data.pvFc{dataid}(:) - pvFc_model(:)).^2);
    end
    
end