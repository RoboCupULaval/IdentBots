function data = parsecsv_batch(filenames, mode)
    
    data = struct();
    for i = 1:numel(filenames)
        [data.t{i}, data.u{i}, data.y{i}] = parsecsv(filenames{i}, mode);
    end
    
end