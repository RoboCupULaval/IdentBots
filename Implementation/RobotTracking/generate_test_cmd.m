
cmd = [zeros(300, 2), 0.1*ones(300,1)];
cmd = cumsum(cmd);

csvwrite('cmd.csv', cmd)