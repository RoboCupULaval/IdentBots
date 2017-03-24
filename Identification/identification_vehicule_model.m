
filenames = {'test01_G05_open_loop.csv'};
exp_data = parsecsv_batch(filenames, 'open_loop');
ident_data.dt = 1/20;
ident_data.w = exp_data.y;
ident_data.v = exp_data.u;

%%

m = 2.2;
L = 0.085;
tau_moteur = 0.2;
gain_moteur = 2*pi;
J = 0.5*m*L^2;
r = 0.025;
gear_ratio = 3.2;

theta = (1:4)*pi/2-pi/4;
Mc = [-cos(theta') -sin(theta') L*ones(4,1)];
Mm = diag([1/m 1/m 1/J]);

Gb = J/tau_moteur;
Ga = gain_moteur*Gb;

Ma = Ga*eye(4);
Mb = Gb*eye(4);

M1 = Mc*Mm*Mc'*Ma/r;
M2 = Mc*Mm*Mc'*Mb/r;

%%

x_min = -10*ones(32,1) ;
x_max = 10*ones(32,1) ;
x0 = ones(32,1);

lb = x_min;
ub = x_max ; 
options = optimset('Display', 'off', 'MaxFunEvals', 1e4);
               
model = @(x) vehicule_model(x, M1, M2, ident_data);
[x_out ,fval, exitflag, output] = fmincon(model, x0, [],[],[],[], lb, ub, [], options);

M1 = M1.*reshape(x_out(1:16)' ,4,4);
M2 = M2.*reshape(x_out(17:32)',4,4);

%% Validation

dt = ident_data.dt;
dataid = 3;

w_model = zeros(size(ident_data.w{dataid}));
v = ident_data.v{dataid};
w_model(1) = ident_data.w{dataid}(1);
for k = 2:length(w_model)
    alpha = M1*v(k-1,:)' - M2*w_model(k-1,:)';
    w_model(k,:) = w_model(k-1,:)' + dt*alpha;
end

nw = 3;
plot(w_model(:,nw)), hold on
plot(ident_data.w{dataid}(:,nw)), hold off