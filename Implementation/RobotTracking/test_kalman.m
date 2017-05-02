clear all;
%close all;
clc;

[t, u, yy] = parsecsv('out2.csv', 'close_loop');

dt = 0.05;
F = [1,  0,  dt, 0,  0,  0;  % pos x
     0,  1,  0,  dt, 0,  0;  % pos y
     0,  0,  1,  0,  0,  0;  % speed x
     0,  0,  0,  1,  0,  0;  % speed y
     0,  0,  0,  0,  1,  dt; % orientation
     0,  0,  0,  0,  0,  1]; % angular speed
 
 H = [1, 0, 0, 0, 0, 0;
      0, 1, 0, 0, 0, 0;
      0, 0, 0, 0, 1, 0];
  
 %vx = 2 + randn(100,1)/10;
 %vy = 3 + randn(100,1)/10;
 %vt = 10 + randn(100,1)/10;
 
f = 0.1;
t = 0:1/20:10;
vx = 0.3*sin(2*pi*f*t);
vy = 0.3*cos(2*pi*f*t);
vt = -0.1*t;
 
 mesures(1,:) = [0;0;0];
 for i = 2:length(t)
    mesures(i,1) = mesures(i-1,1) + vx(i)*dt;
    mesures(i,2) = mesures(i-1,2) + vy(i)*dt;
    mesures(i,3) = mesures(i-1,3) + vt(i)*dt;
 end
 mesures(:,3) = wrapToPi(mesures(:,3));

% mesures = yy(:,4:6); 
%plot(mesures);

 %%
 P = 1e4*eye(6);
 R = 1*eye(3);
 Q = diag([1e-3, 1e-3, 1e-4, 1e-4, .1, .1]);
 x = zeros(6,1);

 for i = 1:length(mesures)
     % Correction
     S = H*P*H'+R;
     K = P*H'*S^(-1);

     y = mesures(i,:)' - H*x;
     y(3) = wrapToPi(y(3));
     x = x + K*y;
     P = (eye(6) - K*H)*P;
     % Prédiction
     x = F*x;
     P = F*P*F'+Q;
     filtre(i,:) = x;
 end

plot(2:length(mesures)+1, wrapToPi(filtre(:,5))), hold on
plot((mesures(:,3)))
plot(filtre(:,6))
plot(vt), hold off