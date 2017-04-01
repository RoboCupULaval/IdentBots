
filenames = {'Data/test_G03_midspeed_1_good.csv',...
             'Data/test_G03_midspeed_2_good.csv',...
             'Data/test_G03_midspeed_3_good.csv',...
             'Data/test_G03_midspeed_4_good.csv',...
             'Data/test_G03_midspeed_5_good.csv'};
exp_data = parsecsv_batch(filenames, 'open_loop_legacy');
ident_data.dt = 1/20;
ident_data.w = exp_data.y;
ident_data.v = exp_data.u;

%%

m = 2.2;
L = 0.085;
tau_moteur = 0.1;
gain_moteur = 120;
J = 0.5*m*L^2;
r = 0.025;
gear_ratio = 3.2;

theta = (1:4)*pi/2-pi/4;
Mc = [-sin(theta') cos(theta') L*ones(4,1)];
Mm = diag([1/m 1/m 1/J]);

Gb = J/tau_moteur;
Ga = gain_moteur*Gb;

Ma = Ga*eye(4);
Mb = Gb*eye(4);

M1 = Mc*Mm*Mc'*Ma/r;
M2 = Mc*Mm*Mc'*Mb/r;

%%

x_min = 0.1*ones(32,1) ;
x_max = 2*ones(32,1) ;
x0 = ones(32,1);

lb = x_min;
ub = x_max ; 
               
model = @(x) vehicule_model(x, M1, M2, ident_data);
[x_out ,fval, exitflag, output] = fmincon(model, x0, [],[],[],[], lb, ub, []);

M1 = M1.*reshape(x_out(1:16)' ,4,4);
M2 = M2.*reshape(x_out(17:32)',4,4);

%% Validation


filenames = {'Data/test_G03_midspeed_1_good.csv'};
exp_data = parsecsv_batch(filenames, 'open_loop_legacy');
valid_data.dt = 1/20;
valid_data.w = exp_data.y;
valid_data.v = exp_data.u;

dt = valid_data.dt;
dataid = 1;
Mc
w_model = zeros(size(valid_data.w{dataid}));
v = valid_data.v{dataid};
w_model(1) = valid_data.w{dataid}(1);
for k = 2:length(w_model)
    alpha = M1*v(k,:)' - M2*w_model(k-1,:)';
    w_model(k,:) = w_model(k-1,:)' + dt*alpha;
end

subplot(4,1,1)
plot(w_model(:,1)), hold on
%plot(100*v(:,1))
plot(valid_data.w{dataid}(:,1)), hold off

subplot(4,1,2)
plot(w_model(:,2)), hold on
%plot(100*v(:,2))
plot(valid_data.w{dataid}(:,2)), hold off

subplot(4,1,3)
plot(w_model(:,3)), hold on
%plot(100*v(:,3))
plot(valid_data.w{dataid}(:,3)), hold off

subplot(4,1,4)
plot(w_model(:,4)), hold on
%plot(100*v(:,4))
plot(valid_data.w{dataid}(:,4)), hold off

