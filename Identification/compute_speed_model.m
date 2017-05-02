function fixed_speed = compute_speed_model(robot_speed, rotation, M3, dt)

    N = length(robot_speed);
    rotF2V = @(phiFc) [cos(phiFc) sin(phiFc) 0 ; -sin(phiFc) cos(phiFc) 0; 0 0 1];
    fixed_speed = zeros(size(robot_speed));
    
    for k = 2:N
        alpha = diag(M3) * ( fixed_speed(k-1,:)' - rotF2V(rotation(k-1))\robot_speed(k-1, :)' );
        fixed_speed(k,:) = fixed_speed(k-1,:)' + dt*alpha;
    end
    
end