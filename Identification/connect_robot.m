

% hwinfo = instrhwinfo('Bluetooth');

b = Bluetooth('RobotDebugFC1891', 1);


filename = 'test_G03_midspeed';

nn = 1;
new_filename = filename;
name_changed = false;
while exist([new_filename, '.csv'], 'file')
    name_changed = true;
    new_filename = [filename, '_', num2str(nn)];
    nn = nn + 1;
end
filename = [new_filename, '.csv'];
if name_changed
    disp('This filename already exist')
    disp(['The filename was changed to ', filename])
end


%%
nline = 1;
timeout = 1000;

dat = struct();
dat.time = zeros(1,1);
dat.robotID = zeros(1,1);
dat.battVoltage = zeros(1,1);
dat.speed = zeros(4,1);
dat.cmd = zeros(4,1);

cell2num = @(c) str2num(cell2mat(c));
try
    fopen(b);
    disp('Acquisition starts')
    %generate_wheel_cmd;
    system('python ../../RobotMCU/pyhermes/pyhermes.py ctrl_test open_loop cmd.csv 2')


    while b.BytesAvailable

        line = strsplit(fgetl(b),'|');

        dat_type = line(1);
        if ~strcmp(dat_type, 'DATA') || length(line) ~= 13, continue, end

        dat.time(nline) = cell2num(line(2));
        dat.robotID(nline) = cell2num(strrep(line(4),'B',''));
        dat.battVoltage(nline) = cell2num(strrep(line(4),'B',''));
        for j = 1:4
            dat.cmd(j, nline)   = cell2num(line(3+2*j));
            dat.speed(j, nline) = cell2num(line(4+2*j));
        end

        nline = nline + 1;

    end

    fclose(b);
catch me
    disp(me)
    fclose(b);
end

dat.time(dat.time > 0) = dat.time(dat.time > 0)-dat.time(1);

disp('Acquisition ended')
plot(dat.time)


%%

c = {'time','motor1_cmd','motor2_cmd','motor3_cmd','motor4_cmd','motor1_speed','motor2_speed','motor3_speed','motor4_speed'};

csvwrite(filename, '')
fid = fopen(filename, 'w');
fprintf(fid, '%s,', c{1,1:end-1}) ;
fprintf(fid, '%s\n', c{1,end}) ;
fclose(fid) ;
dlmwrite(filename, [dat.time', dat.cmd', dat.speed'], '-append') ;