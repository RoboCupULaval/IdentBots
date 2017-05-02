
f = 0.1;
t = 0:1/20:10;
cmd = [0.3*ones(size(t))', 0.3*ones(size(t))', 0.3*ones(size(t))'];

csvwrite('cmd.csv', cmd)