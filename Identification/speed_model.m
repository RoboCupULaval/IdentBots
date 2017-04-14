function J = speed_model(x, M3, ident_data)

    rotF2V = @(phiFc) [cos(phiFc) sin(phiFc) 0 ; -sin(phiFc) cos(phiFc) 0; 0 0 1];
    dt = ident_data.dt;
    Ndata = numel(ident_data.pvFc);

    M3 = M3 .* x;
    
    J = 0;
    for dataid = 1:Ndata           
        
        pvFc_model = zeros(size(ident_data.pvFc{dataid}));
        v = ident_data.v{dataid};
        pvFc_model(ident_data.retard) = ident_data.pvFc{dataid}(ident_data.retard);
        for k = (ident_data.retard+1):length(pvFc_model)
            alpha = diag(M3)*pvFc_model(k-1,:)' - diag(M3)/rotF2V(ident_data.ppFc{dataid}(k-1, 3))*v(k-ident_data.retard, :)';
            pvFc_model(k,:) = pvFc_model(k-1,:)' + dt*alpha;
        end
        J = J + sum((ident_data.pvFc{dataid}(:) - pvFc_model(:)).^2);
    end
    
end