
path = './Data/Delta_2/';

files = dir([path, '*.log']);
filenames = {files.name};

cell2num = @(c) str2double(cell2mat(c));


for i = 1:length(filenames)

    [~, filename, extension] = fileparts(filenames{i});    
    fullname = [path, filename, extension];
    
    nline = 1;

    dat = struct();
    dat.time = zeros(1,1);
    dat.robotID = zeros(1,1);
    dat.battVoltage = zeros(1,1);
    dat.speed = zeros(4,1);
    dat.cmd = zeros(4,1);
    
    try
        fid = fopen(fullname, 'r');
        disp(['parsing ', fullname])

        while true
            line = fgetl(fid);
            if ~ischar(line), break, end
            line = strsplit(line,'|');

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

        fclose(fid);

    catch me
        disp(me)
        fclose(fid);
    end

    dat.time(dat.time > 0) = dat.time(dat.time > 0)-dat.time(1);

    disp('Parsing ended')
    plot(dat.time)


    %%

    c = {'time','motor1_cmd','motor2_cmd','motor3_cmd','motor4_cmd','motor1_speed','motor2_speed','motor3_speed','motor4_speed'};
    
    new_filename = [path, filename, '.csv'];
    
    csvwrite(new_filename, '')
    fid = fopen(new_filename, 'w');
    fprintf(fid, '%s,', c{1,1:end-1}) ;
    fprintf(fid, '%s\n', c{1,end}) ;
    fclose(fid) ;
    dlmwrite(new_filename, [dat.time', dat.cmd', dat.speed'], '-append') ;
    
    pause
    
end
%3,5,11,14