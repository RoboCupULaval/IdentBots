
ident_data = struct();
ident_data.dt = 1/20;
ident_data.w{1} = randn(100,4);
ident_data.v{1} = randn(100,4);

%%

m = 2.2;
L = 0.085;
tau_moteur = 0.01;
gain_moteur = 2*pi*6;
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

x_min = 0.1*ones(32,1) ;
x_max = 2*ones(32,1) ;
x0 = ones(32,1);

lb = x_min;
ub = x_max ; 
options = optimset('Display', 'iter');
               
model = @(x) vehicule_model_ident(x, M1, M2, ident_data);
[x_out ,fval, exitflag, output] = fmincon(model, x0, [],[],[],[], lb, ub, [], options);

M1 = M1*reshape(x_out(1:16)' ,4,4);
M2 = M2*reshape(x_out(17:32)',4,4);

% Validation
dt = ident_data.dt;
for dataid = 1:numel(ident_data.w)
    w_model = zeros(size(ident_data.w{dataid}));
    v = zeros(size(ident_data.v{dataid}));
    w_model(1) = ident_data.w{dataid}(1);
    for k = 2:length(w_model)
        alpha = M1*v(k-1,:)' - M2*w_model(k-1,:)';
        w_model(k,:) = w_model(k-1,:)' + dt*alpha;
    end
end

plot(w_model(:,1)), hold on
plot(ident_data.w{1}(:,1)), hold off