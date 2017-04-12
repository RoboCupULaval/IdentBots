clear all
%% Model:

nwheel = 4;

r = 0.025;
L = 0.085;

lambda_speed = -5;
lambda_robot = -10;
irotF2V = @(phiFc) [cos(phiFc) -sin(phiFc) 0 ; sin(phiFc) cos(phiFc) 0; 0 0 1];
theta = (1:nwheel)*pi/2-pi/4;
Mc = [-sin(theta') cos(theta') L*ones(nwheel,1)];

M3 = diag(lambda_robot*ones(1, 3));
M4 = pinv(Mc)*r;
  
%% Controller

dt = 1/30;
N = 50;

Kp = 14*eye(3);
Ki = 50*eye(3);

pvFc_ref = zeros(3,N);
pvFc   = zeros(3,N);
pvFc_m = zeros(3,N);
ppFc = zeros(3,N);

r_w = zeros(nwheel,N);
speed_command = zeros(3,N);
err  = zeros(3,N);
errI = zeros(3,N);
PI_action= zeros(3,N);

pvFc_ref(2:3,:) = 1;

% TODO: ajouter le retard dans l'estimation d'angle
% TODO: Ajouter predicteur de smith

% ppFc: vecteur position referentiel fixe
% pvFc: vecteur vitesse referentiel fixe
% r_w: consigne de vitesse angulaire des roues
% pvFc_m: Consigne de pvFc filtré (1er ordre, constante de temps de lambda)
% pvFc_ref: consigne de vitesse pvFc

for i = 2:N   
    
    ppFc(:,i) = ppFc(:,i-1) + pvFc(:,i-1)*dt; % Robot angle estimation
    pvFc(:,i) = pvFc(:,i-1) + M3*( pvFc(:,i-1) - irotF2V(ppFc(3,i))*speed_command(:,i-1) )*dt; 
    
    pvFc_m(:,i) = pvFc_m(:,i-1).*(1+lambda_speed*dt) - dt*lambda_speed.*pvFc_ref(:,i-1);    

    err(:,i) = pvFc_m(:,i) - pvFc(:,i);
    errI(:,i) = errI(:,i-1) + err(:,i)*dt;
    
    PI_action = Kp*err(:,i) + Ki*errI(:,i);
    speed_command(:,i) = (-M3*irotF2V(ppFc(3,i))) \ ( lambda_speed*(pvFc_m(:,i)-pvFc_ref(:,i)) - M3*pvFc(:,i) + PI_action );         
    
end

% for i = 2:N   
%     
%     ppFc(:,i) = ppFc(:,i-1) + pvFc(:,i-1)*dt; % Robot angle estimation
%     pvFc(:,i) = pvFc(:,i-1) + ( M3*pvFc(:,i-1) - M3*irotF2V(ppFc(3,i))*M4*r_w(:,i-1) )*dt; 
%     
%     pvFc_m(:,i) = pvFc_m(:,i-1).*(1+lambda_speed*dt) - dt*lambda_speed.*pvFc_ref(:,i-1);    
% 
%     err(:,i) = pvFc_m(:,i) - pvFc(:,i);
%     errI(:,i) = errI(:,i-1) + err(:,i)*dt;
%     
%     PI_action = Kp*err(:,i) + Ki*errI(:,i);
%     r_w(:,i) = (-M3*irotF2V(ppFc(3,i))*M4) \ ( lambda_speed*(pvFc_m(:,i)-pvFc_ref(:,i)) - M3*pvFc(:,i) + PI_action );         
%     
% end

figure(3)

t = (1:N)*dt;

subplot(2,2,1)
plot(t, pvFc_m(1,:)), hold on
plot(t, pvFc(1,:)), hold off
legend('w_m','w')

subplot(2,2,2)
plot(t, pvFc_m(2,:)), hold on
plot(t, pvFc(2,:)), hold off
legend('w_m','w')

subplot(2,2,3)
plot(t, pvFc_m(3,:)), hold on
plot(t, pvFc(3,:)), hold off
legend('w_m','w')

subplot(2,2,4)
plot(t, speed_command')