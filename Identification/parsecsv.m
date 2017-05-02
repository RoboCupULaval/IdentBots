function [t, u, y] = parsecsv(filename, mode)
    
    str_idx = @(C, s) find(ismember(C, s)); % return the index of s in C
    
    file = importdata(filename);
    data = file.data;
    headers = file.colheaders;   

    switch(mode)
        case 'open_loop'
            t = data(:,str_idx(headers, 'time'));
            u = zeros(length(data),4);
            y = zeros(length(data),4);
            for nwheel = 1:4    
                u(:,nwheel) = data(:, str_idx(headers, ['motor',num2str(nwheel),'_cmd']));
                y(:,nwheel) = data(:, str_idx(headers, ['motor',num2str(nwheel),'_speed']));
            end
        case 'open_loop_legacy'
            t = data(:,str_idx(headers, 'time'));
            u = zeros(length(data),4);
            y = zeros(length(data),4);
            for nwheel = 1:4    
                u(:,nwheel) = data(:, str_idx(headers, ['motor',num2str(nwheel),'_cmd']));
                y(:,nwheel) = data(:, str_idx(headers, ['motor',num2str(nwheel),'_speed'])).*2*pi*20/(2048*2*3.2);
            end
        case 'close_loop'
            t = data(:,str_idx(headers, 'time'));
            t = t - min(t);
            u = zeros(length(data),3);
            y = zeros(length(data),6);
            speed_header = 'xyt';
            for i = 1:3    
                u(:,i) = data(:, str_idx(headers, ['cmd_v', speed_header(i)]));
                y(:,i) = data(:, str_idx(headers, ['v', speed_header(i)]));
                y(:,i+3) = data(:, str_idx(headers, ['p', speed_header(i)]));
            end
            y(:,1) = y(:,1)/1000; % Error in python code
            y(:,2) = y(:,2)/1000; % Error in python code
            %y(:,6) = y(:,6)*1000; % Error in python code
        otherwise
            error('parsecsv:InvalidMode', 'The mode specified is invalid')
        
    end
end