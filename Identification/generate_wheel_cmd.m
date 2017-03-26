%% Paramètres
ncommandes = 10;
N = 1000;
longueur_acquisition_50ms = 10000/N;

%% Génération aléatoire
ntransitions = ncommandes-1;
transitions = [ones(ntransitions,3);zeros(longueur_acquisition_50ms - ntransitions, 3)];

rnd_seq = zeros(longueur_acquisition_50ms,3);

rnd_seq(:,1) = randperm(longueur_acquisition_50ms, longueur_acquisition_50ms);
rnd_seq(:,2) = randperm(longueur_acquisition_50ms, longueur_acquisition_50ms);
rnd_seq(:,3) = randperm(longueur_acquisition_50ms, longueur_acquisition_50ms);

rnd_trans = transitions(rnd_seq);
instants_trans = zeros(length([1;find(rnd_trans(:,1));longueur_acquisition_50ms]),3);
instants_trans(:,1) = [1;find(rnd_trans(:,1));longueur_acquisition_50ms];
instants_trans(:,2) = [1;find(rnd_trans(:,2));longueur_acquisition_50ms];
instants_trans(:,3) = [1;find(rnd_trans(:,3));longueur_acquisition_50ms];

temps_commandes = instants_trans(2:end,:)-instants_trans(1:end-1,:);
valeur_commandes = rand(100,4)-0.5; % Distribution uniforme autour de 0 (max ±0.5 m/s)

valeur_instant = zeros(longueur_acquisition_50ms,3);
for n = 1:3
    for m = 1:ncommandes
        k = temps_commandes(m,n);
        i = instants_trans(m,n);
        valeur_instant(i:i+k-1,n) = valeur_commandes(m,n)*ones(k,1);
    end
end
rnd_trans_cmd = rnd_trans(:,1) | rnd_trans(:,2) | rnd_trans(:,3);
instants_trans_cmd = [1;find(rnd_trans_cmd);longueur_acquisition_50ms];


t = (instants_trans_cmd(2:end)-instants_trans_cmd(1:end-1))';
t = N*t;

v_cmd = valeur_instant(instants_trans_cmd(1:end-1),:);

L = 0.085;
r = 0.025;

theta = (1:4)*pi/2-pi/4;
Mc = [-sin(theta') cos(theta') L*ones(4,1)];

vx = v_cmd(:,1)';
vy = v_cmd(:,2)';
w =  v_cmd(:,3)';

cmd = (Mc*[vx;vy;w]/r/120)';
cmd(abs(cmd) < 0.05) = 0;
cmd = [t', cmd];


csvwrite('cmd.csv', cmd)