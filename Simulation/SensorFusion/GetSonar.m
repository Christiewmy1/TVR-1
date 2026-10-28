function h = GetSonar()
    persistent sonarAlt
    persistent k firstRun

    if isempty(firstRun)
        
        filename = 'Sensor_Fusion_OR_Data_updated.csv'; %wtv the file name for the data is
       
        %skip comment lines in the csv files using 
        %filetype: text and commentstyle: #
        opts = detectImportOptions(filename, 'FileType', 'text', 'CommentStyle', '#');
        T = readtable(filename, opts);
        
        %time and altitude (adjust column names if needed)
        time = T{:, 1};       %time in seconds
        position = T{:, 2};   %altitude in meters
        
        %export as .mat file, readable for RunFilter.m
        flight_data.time = time;
        flight_data.position = [zeros(length(time), 2), position]; %[X, Y, Z] format (Z = altitude)
        
        save('imu_flight_path.mat', 'flight_data');
        
        sonarAlt = position;
        k = 1;
        firstRun = 1;
    end

    h = sonarAlt(k);
    k = k + 1;
end