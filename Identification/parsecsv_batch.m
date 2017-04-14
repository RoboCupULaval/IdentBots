function data = parsecsv_batch(filenames, mode)
    
    data = struct();
    for i = 1:numel(filenames)
        if strncmp(mode, 'open', 4)
            [data.t{i}, data.u{i}, data.y{i}] = parsecsv(filenames{i}, mode);
        elseif strncmp(mode, 'close', 5)
            [data.t{i}, data.v{i}, data.y{i}] = parsecsv(filenames{i}, mode);
            data.pvFc{i} = data.y{i}(:,1:3);
            data.ppFc{i} = data.y{i}(:,4:6);
        end
    end
    
end