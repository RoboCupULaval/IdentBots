clear all;
close all;
clc;

dt = 0.033;
F = [1,0,dt,  0,0,0;
     0,1, 0, dt,0,0;
     0,0, 1,  0,0,0;
     0,0, 0, 1, 0,0;
     0,0, 0, 0, 1,dt;
     0,0, 0, 0, 0,1];
 H = [1,0,0,0,0,0;
      0,1,0,0,0,0;
      0,0,0,0,1,0];
 vx = 2 + randn(100,1)/10;
 vy = 3 + randn(100,1)/10;
 vt = 20 + randn(100,1)/100;
 mesures(1,:) = [0;0;0];
 for i = 2:100
    mesures(i,1) = mesures(i-1,1) + vx(i)*dt;
    mesures(i,2) = mesures(i-1,2) + vy(i)*dt;
    mesures(i,3) = mesures(i-1,3) + vt(i)*dt;
 end
 mesures(:,3) = wrapToPi(mesures(:,3));
 figure; plot(mesures);
 
 P = zeros(6);
 R = 0.01*eye(3);
 Q = 0.01*eye(6);
 x = zeros(6,1);
 for i = 1:100
 % Correction
 S = H*P*H'+R;
 K = P*H'*S^(-1);
 y = mesures(i,:)' - H*x;
 y(3) = wrapTo2Pi(y(3));
 x = x + K*y;
 P = (eye(6) - K*H)*P;
 % Prédiction
 x = F*x;
 P = F*P*F'+Q;
 filtre(i,:) = x;
 end

 figure; plot(filtre)