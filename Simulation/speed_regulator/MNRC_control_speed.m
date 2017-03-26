clear all
%% Model:

nwheel = 4;

r = 0.025;
L = 0.085;

lambda_w = -10; % tau_w = -1/lambda_w
rotF2V = @(phiFc) [cos(phiFc) sin(phiFc) 0 ; -sin(phiFc) cos(phiFc) 0; 0 0 1];
theta = (1:nwheel)*pi/2-pi/4;
Mc = [-cos(theta') -sin(theta') L*ones(nwheel,1)];

M3 = diag(lambda_w*ones(1,3));
M4 = pinv(Mc)./r;

%% Controller

dt = 1/20;

Kp = 15*eye(nwheel);
Ki = 40*eye(nwheel);

gamma = -1/0.2*eye(nwheel);
invM1 = inv(M1);

N = 1000;

pvFc_ref = ones(nwheel,N);
w_ref(1,:) = w_ref(1,:)*-0.8; 
w    = zeros(nwheel,N);
w_m  = zeros(nwheel,N);
u    = zeros(nwheel,N);
err  = zeros(nwheel,N);
errI = zeros(nwheel,N);

