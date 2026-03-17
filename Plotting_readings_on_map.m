clear;
clc;

u = udpport("datagram","IPV4","LocalPort",50001);
disp("Listening on port 50001...");

figure(1);

map = occupancyMap(12, 12, 20);   % 12 m x 12 m, 20 cells/m
sensorPose = [0 0 0];
maxInsertRange = 10;
figure(2);

while true
    if u.NumDatagramsAvailable == 0
        pause(0.01);
        continue;
    end

    % Read one UDP datagram as raw bytes
    dg = read(u, 1, "uint8");
    rawData = uint8(dg.Data);

    if numel(rawData) < 2
        continue;
    end

    % First 2 bytes = count (uint16)
    count = double(typecast(rawData(1:2), 'uint16'));

    % Expected packet size:
    % 2 bytes count + 2*count bytes angles + 2*count bytes distances
    expectedBytes = 2 + 2*count + 2*count;

    if numel(rawData) < expectedBytes
        warning("Incomplete packet: got %d bytes, expected %d", ...
            numel(rawData), expectedBytes);
        continue;
    end

    measurementBytes = rawData(3:expectedBytes);

    % Unpack angles and distances
    anglesRaw = typecast(measurementBytes(1:2*count), 'uint16');
    distRaw   = typecast(measurementBytes(2*count+1:4*count), 'uint16');

    % Convert to MATLAB units
    anglesDeg = double(anglesRaw)/100;      % because ESP32 sends x100 degrees
    anglesRad = deg2rad(anglesDeg);     % lidarScan wants radians
    rangesM   = double(distRaw) / 1000; % sender distance is in mm

    % Filter invalid points
    valid = rangesM > 0.05 & rangesM < 20 & ...
            isfinite(rangesM) & isfinite(anglesRad);

    anglesRad = anglesRad(valid);
    rangesM   = rangesM(valid);

    if numel(rangesM) < 5
        continue;
    end

    % Sort by angle for nicer plotting
    [anglesRad, idx] = sort(anglesRad);
    rangesM = rangesM(idx);


    scan = lidarScan(rangesM, anglesRad);
    insertRay(map, sensorPose, scan, maxInsertRange);
    
    figure(2);
    show(map);
    title('Local Occupancy Map');
    drawnow limitrate;

    % Cartesian plot
    x = rangesM .* cos(anglesRad);
    y = rangesM .* sin(anglesRad);

    clf;
    plot(x, y, '.');
    hold on;
    plot(0,0,'ro','MarkerSize',8,'LineWidth',2);
    axis equal;
    grid on;
    xlabel('X [m]');
    ylabel('Y [m]');
    title(sprintf('Stationary RPLidar Scan (%d points)', numel(rangesM)));
    drawnow;
end