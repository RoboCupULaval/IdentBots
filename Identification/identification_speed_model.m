
%% Load data

filenames = {'Data\Robot_fixed_speed\grsim_test3.csv'};
ident_data = parsecsv_batch(filenames, 'close_loop');
ident_data.dt = 1/30;

ident_data.retard = 1; % Delay in sample number

%% Initial guess

lambda_speed = -10; 
M3 = lambda_speed*ones(3,1);

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

filenames = {'Data\Robot_fixed_speed\grsim_test3.csv'};
model_data = parsecsv_batch(filenames, 'close_loop');
model_data.dt = 1/30;
model_data.retard = 1; % Delay in sample number

%
% acc = lambda*(robot_speed - robot2fixed(orientation)*speed_command)
% speed_command = (-M3*irotF2V(ppFc(3,i))) \ ( lambda_speed*(pvFc_m-pvFc_ref) - M3*pvFc + correction );         
% -M3*robot2fixed(orientation)*speed_command = lambda_speed*(filtered_ref - ref) - M3*robot_speed + correction

for dataid = 1
    
    model_data.pvFc_model{dataid} = zeros(size(ident_data.pvFc{dataid}));
    model_data.ppFc_model{dataid} = zeros(size(ident_data.ppFc{dataid}));
    model_data.v{dataid} = ident_data.v{dataid};
    
    model_data.pvFc_model{dataid}(ident_data.retard,:) = ident_data.pvFc{dataid}(ident_data.retard,:);
    %validation_data.ppFc_model{dataid}(ident_data.retard,:) = ident_data.ppFc{dataid}(ident_data.retard,:);
    for k = (ident_data.retard+1):length(model_data.pvFc_model{dataid})
        alpha = diag(M3)*model_data.pvFc_model{dataid}(k-1,:)' -...
                diag(M3)/rotF2V(ident_data.ppFc{dataid}(k-1, 3))*model_data.v{dataid}(k-ident_data.retard,:)';
        model_data.pvFc_model{dataid}(k,:) = model_data.pvFc_model{dataid}(k-1,:)' + dt*alpha;
    end
    
end

%%

plot(model_data.pvFc_model{1}(:,3)), hold on
plot(ident_data.pvFc{1}(:,3)), hold off