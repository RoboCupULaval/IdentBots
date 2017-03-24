function [t, u, y] = parsecsv(filename, mode)
    
    str_idx = @(C, s) find(ismember(C, s)); % return the index of s in C
    
    file = importdata(filename);
    data = file.data;
    headers = file.colheaders;
    
    u = zeros(length(data),4);
    y = zeros(length(data),4);

    switch(mode)
        case 'open_loop'
            t = data(:,str_idx(headers, 'time'));
            for nwheel = 1:4    
                u(:,nwheel) = data(:, str_idx(headers, ['motor',num2str(nwheel),'_cmd']));
                y(:,nwheel) = data(:, str_idx(headers, ['motor',num2str(nwheel),'_speed'])).*2*pi/(2048*2*3.2);
            end
        otherwise
            error('parsecsv:InvalidMode', 'The mode specified is invalid')
        
    end
end