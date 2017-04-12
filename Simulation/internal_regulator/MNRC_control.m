clear all
%% Model:

nwheel = 4;

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

Ma = Ga*eye(nwheel);
Mb = Gb*eye(nwheel);

M1 = Mc*Mm*Mc'*Ma/r;
M2 = Mc*Mm*Mc'*Mb/r;


M1 = [832.5160 , 348.0510 , 150.7291 , 327.6491;
      486.8178 , 692.4930 , 399.4206 , 117.5622;
      241.0265 , 239.6313 , 761.3489 , 235.5609;
      221.2959 ,  59.1184 , 116.8171 , 702.2684];

M2 = [4.5265  ,  0.2949  ,  1.1064  ,  0.3471;
      1.0369  ,  2.9667  ,  0.8380  ,  0.1454;
      0.1446  ,  0.2892  ,  2.9959  ,  0.2894;
      0.7098  ,  0.1451  ,  0.2896  ,  3.3578];

%% Controller

dt = 1/20;

Kp = 7*eye(nwheel);
Ki = 25*eye(nwheel);

lambda = -1/0.5;

N = 1000;

w_ref = zeros(nwheel,N);
w_ref(1:3,100:500) = 15;
w_ref(3:4,300:600) = 15;

w    = zeros(nwheel,N);
w_m  = zeros(nwheel,N);
u    = zeros(nwheel,N);
err  = zeros(nwheel,N);
errI = zeros(nwheel,N);

for i = 2:N
          
    w_m(:,i) = w_m(:,i-1).*(1+dt*lambda) - dt*lambda.*w_ref(:,i); % filtered tracking reference    
    w(:,i) = w(:,i-1) + (M1*u(:,i-1)-M2*w(:,i-1))*dt; 
    
    err(:,i) = w_m(:,i) - w(:,i); % Tracking error
    errI(:,i) = errI(:,i-1) + err(:,i)*dt;
    
    PI_action = Kp*err(:,i) + Ki*errI(:,i);
    u(:,i) = M1\( lambda*(w_m(:,i)-w_ref(:,i)) + M2*w(:,i) + PI_action);
    %u(u(:,i) > 1,i) = 1;
    %u(u(:,i) < -1,i) = -1;

end


figure(3)

t = (1:N)*dt;

subplot(2,2,1)
plot(t, w_m(1,:)), hold on
plot(t, w(1,:)), hold off
legend('w_m','w')

subplot(2,2,2)
plot(t, w_m(2,:)), hold on
plot(t, w(2,:)), hold off
legend('w_m','w')

subplot(2,2,3)
plot(t, w_m(3,:)), hold on
plot(t, w(3,:)), hold off
legend('w_m','w')

subplot(2,2,4)
plot(t, w_m(4,:)), hold on
plot(t, w(4,:)), hold off
legend('w_m','w')