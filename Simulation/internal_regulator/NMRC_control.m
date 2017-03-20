clear all
%% Model:

nwheel = 3;
m = 2.2;
L = 0.085;
tau_moteur = 0.15;
gain_moteur = 2*pi*6;
J = 0.5*m*L^2;
r = 0.025;
gear_ratio = 3.2;

theta = (1:nwheel)*pi/2-pi/4;
Mc = [-cos(theta') -sin(theta') L*ones(nwheel,1)];
Mm = diag([1/m 1/m 1/J]);

Gb = J/tau_moteur;
Ga = gain_moteur*Gb;

Ma = Ga*eye(nwheel);
Mb = Gb*eye(nwheel);

M1 = Mc*Mm*Mc'*Ma/r;
M2 = Mc*Mm*Mc'*Mb/r;

%% Controller

dt = 1/20;

Kp = 15*eye(nwheel);
Ki = 40*eye(nwheel);

gamma = -1/0.2*eye(nwheel);
invM1 = inv(M1);

N = 1000;

w_ref = ones(nwheel,N);
w_ref(1,:) = w_ref(1,:)*-0.8; 
w    = zeros(nwheel,N);
w_m  = zeros(nwheel,N);
u    = zeros(nwheel,N);
err  = zeros(nwheel,N);
errI = zeros(nwheel,N);

for i = 2:N
    
    w(:,i) = w(:,i-1) + (M1*u(:,i-1)-M2*w(:,i-1))*dt;
    
    w_m(:,i) = (eye(nwheel)+dt*gamma)*w_m(:,i-1) - dt*gamma*w_ref(:,i-1); % Model estimate

    err(:,i) = w_m(:,i) - w(:,i); % Tracking error
    errI(:,i) = errI(:,i-1) + err(:,i)*dt;
    
    PI_action = Kp*err(:,i) + Ki*errI(:,i);
    u(:,i) = invM1*( gamma*(w(:,i)-w_ref(:,i)) - M2*w(:,i) + PI_action);
    
end


figure(3)

idx = 1:N;
t = (1:N)*dt;
plot(t, u(1,idx)), hold on
plot(t, w_m(1,idx))
plot(t, w(1,idx)), hold off
legend('u','w_m','w','PI')