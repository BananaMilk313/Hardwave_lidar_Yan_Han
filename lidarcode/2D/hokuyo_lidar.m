function lidarRealtime3View
    clear; clc; close all;

    node = ros2node("/matlab_node");
    scanSubscriber = ros2subscriber(node, "/scan", "sensor_msgs/LaserScan");

    minRange = 0.5;    
    maxRange = 5.0;   
    minAngleDeg = -90; 
    maxAngleDeg = 90; 

    clusterDistanceThreshold = 0.1;  
    minClusterPoints = 10;         
    minClusterSize = 0.1;          

    xLimits = [-5, 5];   
    yLimits = [-5, 5];
    gridSpacing = 0.01;  

    disp("等待接收 2D 激光雷达数据...");

    hFig = figure('Name','Real-time Laser Scan Processing','NumberTitle','off',...
                  'Units','normalized','Position',[0.05 0.05 0.9 0.8]);

    while isvalid(hFig)
        frameTimer = tic; 

        if ~isempty(scanSubscriber.LatestMessage)
            scanMsg = scanSubscriber.LatestMessage;

            angleMin = double(scanMsg.angle_min);
            angleInc = double(scanMsg.angle_increment);
            ranges   = double(scanMsg.ranges);
            numPoints = length(ranges);

            anglesRad = (angleMin : angleInc : (angleMin + (numPoints-1)*angleInc))';
            x = ranges .* cos(anglesRad);
            y = ranges .* sin(anglesRad);
            z = zeros(size(x));

            validMask = isfinite(ranges) & (ranges > 0);
            x = x(validMask);
            y = y(validMask);
            z = z(validMask);
            anglesRad = anglesRad(validMask);
            ranges    = ranges(validMask);

            anglesDeg = rad2deg(anglesRad);
            validIndices = (ranges >= minRange) & (ranges <= maxRange) & ...
                           (anglesDeg >= minAngleDeg) & (anglesDeg <= maxAngleDeg);
            filteredX = x(validIndices);
            filteredY = y(validIndices);
            filteredZ = z(validIndices);

            if ~isempty(filteredX)
                pc = pointCloud([filteredX, filteredY, filteredZ]);

                clf(hFig);
                subplot(2,2,1);
                pcshow(pc, 'VerticalAxis','Z','VerticalAxisDir','Up');
                title(sprintf('1) 过滤后的点云\nRange: %.1f ~ %.1f m', minRange, maxRange));
                xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

                [labels, numClusters] = pcsegdist(pc, clusterDistanceThreshold);
                subplot(2,2,2);
                pcshow(pc.Location, labels);
                if numClusters > 0
                    colormap(hsv(numClusters));
                end
                title(sprintf('2) 聚类结果, 簇数 = %d', numClusters));
                xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

                obstaclePoints = [];
                for i = 1 : numClusters
                    clusterPts = pc.Location(labels == i, :);
                    clusterSize = max(clusterPts, [], 1) - min(clusterPts, [], 1);
                    if (size(clusterPts,1) >= minClusterPoints) && all(clusterSize >= minClusterSize)
                        obstaclePoints = [obstaclePoints; clusterPts];
                    end
                end

                subplot(2,2,3);
                plot(filteredX, filteredY, '.');
                axis equal;
                xlim(xLimits); ylim(yLimits);
                grid on;
                title('3) 俯视图 (Top-Down View)');
                xlabel('X (m)'); ylabel('Y (m)');

                subplot(2,2,4);
                axis off;
                procTime = toc(frameTimer);
                txt = sprintf('激光帧处理耗时：%.3f s\n过滤后点数：%d\n聚类数量：%d', ...
                               procTime, length(filteredX), numClusters);
                text(0.1, 0.5, txt, 'FontSize', 12);
            end
        end

        drawnow;
        pause(0.03);
    end

    disp("程序结束或窗口被关闭。");
end
