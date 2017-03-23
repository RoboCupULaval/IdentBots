
%% Load data

ident_data = struct();
ident_data.dt = 1/20;
ident_data.nwheel = 4;
ident_data.rw{1} = randn(100,4);   % Wheel speed command
ident_data.pvFc{1} = randn(100,3); % Speed of the center of mass of the robot in the fixed frame [vx, vy, omega]
ident_data.ppFc{1} = randn(100,3); % Speed of the center of mass of the robot in the fixed frame [vx, vy, omega]
ident_data.retard = 10; % Delay in sample number

%% Initial guess

r = 0.025;
L = 0.085;
nwheel = ident_data.nwheel;

lambda_w = -10; % tau_w = -1/lambda_w
rotF2V = @(phiFc) [cos(phiFc) sin(phiFc) 0 ; -sin(phiFc) cos(phiFc) 0; 0 0 1];
theta = (1:nwheel)*pi/2-pi/4;
Mc = [-cos(theta') -sin(theta') L*ones(nwheel,1)];

M3 = diag(lambda_w*ones(1,3));
M4 = pinv(Mc)./r;

%% Identification

x_min = 0.01*ones(9+3*nwheel,1) ;
x_max = 20*ones(9+3*nwheel,1) ;
x0 = ones(9+3*nwheel,1);

lb = x_min;
ub = x_max ; 
options = optimset('Display', 'iter');
               
model = @(x) speed_model(x, M3, M4, ident_data);
[x_out ,fval, exitflag, output] = fmincon(model, x0, [],[],[],[], lb, ub, [], options);

M3 = M3.*reshape(x_out(1:9)', 3, 3);
M4 = M4.*reshape(x_out(10:end)', 3, ident_data.nwheel);

%% Validation

dt = ident_data.dt;
model_data = struct();
for dataid = 1
    
    model_data.pvFc_model{dataid} = zeros(size(ident_data.pvFc{dataid}));
    model_data.ppFc_model{dataid} = zeros(size(ident_data.ppFc{dataid}));
    model_data.rw{dataid} = ident_data.rw{dataid};
    
    model_data.pvFc_model{dataid}(ident_data.retard,:) = ident_data.pvFc{dataid}(ident_data.retard,:);
    %validation_data.ppFc_model{dataid}(ident_data.retard,:) = ident_data.ppFc{dataid}(ident_data.retard,:);
    for k = (ident_data.retard+1):length(model_data.pvFc_model{dataid})
        alpha = M3*model_data.pvFc_model{dataid}(k-1,:)' -...
                M3/rotF2V(ident_data.ppFc{dataid}(k-1, 3))*M4*model_data.rw{dataid}(k-ident_data.retard,:)';
        model_data.pvFc_model{dataid}(k,:) = model_data.pvFc_model{dataid}(k-1,:)' + dt*alpha;
    end
    
end

%%

plot(model_data.pvFc_model{1}(:,1)), hold on
plot(ident_data.pvFc{1}(:,1)), hold on