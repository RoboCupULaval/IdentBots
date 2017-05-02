function J = speed_model(x, M3, ident_data)

    dt = ident_data.dt;
    M3 = M3 .* x;
    J = 0;
    for dataid = 1:numel(ident_data.pvFc)           
        fixed_speed_ident = ident_data.pvFc{dataid};
        robot_speed = ident_data.v{dataid};
        rotation = ident_data.ppFc{dataid}(:,3);
        fixed_speed = compute_speed_model(robot_speed, rotation, M3, dt);
        J = J + sum( (fixed_speed_ident(:) - fixed_speed(:)).^2 );
    end
    
end