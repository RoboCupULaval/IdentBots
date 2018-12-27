
files = dir('./Data/Delta_2/*.csv');
filenames = {files.name};
filefolder = {files.folder};
for i = 1:length(filenames)
   filenames{i} = [filefolder{i}, '/', filenames{i}];  
end
exp_data = parsecsv_batch(filenames, 'open_loop');
ident_data.dt = 1/100;
ident_data.w = exp_data.y;
ident_data.v = exp_data.u;

%%

m = 3.0;
L = 0.0785;
tau_moteur = 0.005;
gain_moteur = 100;
J = 0.5*m*L^2;
r = 0.033;
gear_ratio = 5;

theta = wrapTo2Pi(deg2rad([60, 141, -141, -60]));
Mc = [-sin(theta') cos(theta') L*ones(4,1)];
Mm = diag([1/m 1/m 1/J]);

Gb = J/tau_moteur;
Ga = gain_moteur*Gb;

Ma = Ga*eye(4);
Mb = Gb*eye(4);

M1 = Mc*Mm*Mc'*Ma/r;
M2 = Mc*Mm*Mc'*Mb/r;

%%

x_min = 0.01*ones(32,1) ;
x_max = 2*ones(32,1) ;
x0 = ones(32,1);

lb = x_min;
ub = x_max ; 
options = optimset('MaxFunEvals', 10000);               
model = @(x) vehicule_model(x, M1, M2, ident_data);
[x_out ,fval, exitflag, output] = fmincon(model, x0, [],[],[],[], lb, ub, [], options);

M1 = M1.*reshape(x_out(1:16)' ,4,4);
M2 = M2.*reshape(x_out(17:32)',4,4);

ident_ss = ss(-M2, M1, eye(4), zeros(4));
ident_ss = c2d(ident_ss, ident_data.dt);

%% Validation


filenames = {'./Data/Delta_2/test_06.csv'};
exp_data = parsecsv_batch(filenames, 'open_loop');
valid_data.dt = 1/100;
valid_data.w = exp_data.y;
valid_data.v = exp_data.u;

dt = valid_data.dt;
dataid = 1;

U = valid_data.v{dataid};
T = 0:dt:length(valid_data.v{dataid})*dt-dt;
X0 = valid_data.w{dataid}(1,:);
w_model = lsim(ident_ss, U, T, X0);

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

