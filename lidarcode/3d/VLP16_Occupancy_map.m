clear; clc; close all;

node = ros2node("/matlab_node");
pointsSubscriber = ros2subscriber(node, "/velodyne_points", "sensor_msgs/PointCloud2");

minRange = 0.5;
maxRange = 5;
minAngle = -180;
maxAngle = 180;
minDensityThreshold = 10;
maxDensityThreshold = 50;
resolution = 0.1;
xLimits = [-5, 5];
yLimits = [-5, 5];
clusterDistanceThreshold = 0.2;

disp("等待接收点云数据...");
tic;
while toc < 60
    if ~isempty(pointsSubscriber.LatestMessage)
        msg = pointsSubscriber.LatestMessage;
        xyzPoints = rosReadXYZ(msg);
        distances = sqrt(xyzPoints(:,1).^2 + xyzPoints(:,2).^2 + xyzPoints(:,3).^2);
        angles = atan2d(xyzPoints(:,2), xyzPoints(:,1));
        validIndices = (distances >= minRange) & (distances <= maxRange) & ...
                       (angles >= minAngle) & (angles <= maxAngle);
        filteredPoints = xyzPoints(validIndices, :);
        
        if size(filteredPoints,1) > 0
            pc = pointCloud(filteredPoints);
            
            % Figure 1: 3D点云
            figure(1);
            pcshow(pc, 'VerticalAxis', 'Z', 'VerticalAxisDir', 'Up');
            title(['3D点云限制 | 距离: ', num2str(minRange), '-', num2str(maxRange), ...
                   ' 米, 角度: ', num2str(minAngle), '° 到 ', num2str(maxAngle), '°']);
            xlabel('X'); ylabel('Y'); zlabel('Z');
            drawnow;
            
            % Figure 2: 占据图 (基于过滤后点云)
            xEdges = xLimits(1):resolution:xLimits(2);
            yEdges = yLimits(1):resolution:yLimits(2);
            occupancyMap = histcounts2(filteredPoints(:,1), filteredPoints(:,2), xEdges, yEdges);
            figure(2);
            imagesc(xEdges, yEdges, occupancyMap', 'AlphaData', ~isnan(occupancyMap'));
            axis xy; colormap(flipud(gray)); colorbar; set(gca, 'Color', 'w');
            caxis([minDensityThreshold maxDensityThreshold]);
            title('占据图 (Occupancy Map)');
            xlabel('X (米)'); ylabel('Y (米)');
            drawnow;
            
            % Figure 3: 聚类结果
            [labels, numClusters] = pcsegdist(pc, clusterDistanceThreshold);
            figure(3);
            pcshow(pc.Location, labels);
            colormap(hsv(numClusters));
            title(['聚类结果 | 聚类数量: ', num2str(numClusters)]);
            xlabel('X (米)'); ylabel('Y (米)'); zlabel('Z (米)');
            drawnow;
            
            % 障碍物识别
            minClusterPoints = 10;
            minClusterSize = 0.1;
            obstaclePoints = [];
            for i = 1:numClusters
                clusterIndices = (labels == i);
                clusterPoints = pc.Location(clusterIndices, :);
                minVals = min(clusterPoints, [], 1);
                maxVals = max(clusterPoints, [], 1);
                clusterSize = maxVals - minVals;
                if (size(clusterPoints,1) >= minClusterPoints) && all(clusterSize >= minClusterSize)
                    obstaclePoints = [obstaclePoints; clusterPoints];
                end
            end
            
            % Figure 4: 障碍物点云 (3D显示)
            figure(4);
            if ~isempty(obstaclePoints)
                obstaclePc = pointCloud(obstaclePoints);
                pcshow(obstaclePc, 'VerticalAxis', 'Z', 'VerticalAxisDir', 'Up');
                title(['障碍物点云 | 识别数量: ', num2str(size(obstaclePoints,1))]);
                xlabel('X (米)'); ylabel('Y (米)'); zlabel('Z (米)');
            else
                title('未检测到障碍物');
            end
            drawnow;
            
            % Figure 5: 二值矩阵 (基于障碍物点云)
            figure(5);
            if ~isempty(obstaclePoints)
                obstacleOccupancyMap = histcounts2(obstaclePoints(:,1), obstaclePoints(:,2), xEdges, yEdges);
                binaryMatrixObs = obstacleOccupancyMap > 0;
                imagesc(xEdges, yEdges, binaryMatrixObs', 'AlphaData', ~isnan(binaryMatrixObs'));
                axis xy; colormap(flipud(gray)); set(gca, 'Color', 'w');
                title('障碍物二值矩阵 (Binary Matrix from Obstacle Points)');
                xlabel('X (米)'); ylabel('Y (米)');
            else
                title('障碍物二值矩阵 (Binary Matrix from Obstacle Points) - 无障碍物');
            end
            drawnow;
        end
    end
end
