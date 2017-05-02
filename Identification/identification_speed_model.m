
%% Load data

filenames = {'Data\Robot_fixed_speed\real_170417_02.csv'};
ident_data = parsecsv_batch(filenames, 'close_loop');
ident_data.dt = 1/30;

ident_data.retard = 1; % Delay in sample number

%% Initial guess

lambda_speed = -10; 
M3 = lambda_speed*ones(3,1);
rotF2V = @(phiFc) [cos(phiFc) sin(phiFc) 0 ; -sin(phiFc) cos(phiFc) 0; 0 0 1];

%% Identification

x_min = 0.05*ones(3,1) ;
x_max = 2*ones(3,1) ;
x0 = ones(3,1);

lb = x_min;
ub = x_max ; 
options = optimset('Display', 'iter');
               
model = @(x) speed_model(x, M3, ident_data);
[x_out ,fval, exitflag, output] = fmincon(model, x0, [],[],[],[], lb, ub, [], options);

M3 = M3 .* x_out;

%% Validation

dt = ident_data.dt;

filenames = {'Data\Robot_fixed_speed\real_170417_01.csv'};
model_data = parsecsv_batch(filenames, 'close_loop');
model_data.dt = 1/20;
model_data.retard = 1; % Delay in sample number

dataid = 1;
    
model_data.pvFc_model{dataid} = zeros(size(ident_data.pvFc{dataid}));
model_data.ppFc_model{dataid} = zeros(size(ident_data.ppFc{dataid}));
model_data.v{dataid} = ident_data.v{dataid};

fixed_speed_ident = model_data.pvFc{dataid};
robot_speed = model_data.v{dataid};
rotation = model_data.ppFc{dataid}(:,3);
fixed_speed_model = compute_speed_model(robot_speed, rotation, M3, dt);
   

%%

plot(fixed_speed_model(:,1:2)), hold on
plot(ident_data.pvFc{1}(:,1:2)), hold off