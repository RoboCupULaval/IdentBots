N = 100;
r = 0.025;
L = 0.085;

theta = (1:4)*pi/2-pi/4;
Mc = [-cos(theta') -sin(theta') L*ones(4,1)];

f = 0.04;
vx = 0.4*sin(2*pi*f*(randn(1,N)));
vy = 0.4*cos(2*pi*f*(randn(1,N)));
w = zeros(1,N);
t = 100*ones(1,N);

cmd = [t', (Mc*[vx;vy;w]/r/120)'];

csvwrite('cmd.csv', cmd)