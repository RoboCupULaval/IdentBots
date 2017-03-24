N = 15;
r = 0.025;

theta = (1:4)*pi/2-pi/4;
Mc = [-cos(theta') -sin(theta') L*ones(4,1)];

vx = 0.4*(2*rand(1,N)-1);
vy = 0.4*(2*rand(1,N)-1);
w = 0.5*(2*rand(1,N)-1);
t = floor(1000*rand(1,N));
cmd = [t', (Mc*[vx;vy;w]/r/60)'];

csvwrite('..\..\RobotMCU\pyhermes\cmd.csv', cmd)