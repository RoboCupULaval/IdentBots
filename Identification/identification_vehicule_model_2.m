
filenames = {'Data/310317/G05_310317_1.csv',...
             'Data/310317/G05_310317_2.csv',...
             'Data/310317/G05_310317_3.csv',...
             'Data/310317/G05_310317_4.csv',...
             'Data/310317/G05_310317_5.csv'};
exp_data = parsecsv_batch(filenames, 'open_loop');
dt = 1/20;

ident_data = iddata(exp_data.y, exp_data.u, dt); 
%%

nwheel = 4;

m = 2.2;
L = 0.085;
tau_moteur = 0.1;
gain_moteur = 120;
J = 0.5*m*L^2;
r = 0.025;
gear_ratio = 3.2;

theta = (1:4)*pi/2-pi/4;
Mc = [-sin(theta') cos(theta') L*ones(nwheel,1)];
Mm = diag([1/m 1/m 1/J]);

Gb = J/tau_moteur;
Ga = gain_moteur*Gb;

Ma = Ga*eye(nwheel);
Mb = Gb*eye(nwheel);

M1 = Mc*Mm*Mc'*Ma/r;
M2 = Mc*Mm*Mc'*Mb/r;

%%



A = -M2;
B = M1; 
C = eye(nwheel);
D = zeros(nwheel); 
K = zeros(nwheel);
x0 = zeros(nwheel,1);

init_ss = c2d(idss(A,B,C,D,K,x0,0), dt, 'zoh');

init_ss.Structure.a.Free = true;
init_ss.Structure.b.Free = true;
init_ss.Structure.c.Free = false;
init_ss.Structure.d.Free = false;
init_ss.Structure.k.Free = false;



sys = ssest(ident_data, init_ss, 'Focus', 'simulation');
    

