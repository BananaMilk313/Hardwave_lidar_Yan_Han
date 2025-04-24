% 清理环境
clear; clc; close all;

% 创建ROS 2节点并订阅点云话题
node = ros2node("/matlab_node");
pointsSubscriber = ros2subscriber(node, "/velodyne_points", "sensor_msgs/PointCloud2");

% 设置范围限制
minRange = 0.5;
maxRange = 5;

% 设置角度范围限制 (以度为单位)
minAngle = -180;
maxAngle = 180;

% 设置点云密度阈值
minDensityThreshold = 10;
maxDensityThreshold = 50;

% 设置栅格分辨率和统一范围
resolution = 0.1;
xLimits = [-5, 5];
yLimits = [-5, 5];

% 设置聚类距离阈值
clusterDistanceThreshold = 0.2; % 聚类距离阈值，单位为米

disp("等待接收点云数据...");

% 主循环：接收并处理点云数据
tic;
while toc < 120 % 持续运行120秒
    % 检查是否有新消息
    if ~isempty(pointsSubscriber.LatestMessage)
        % 获取最新点云消息
        msg = pointsSubscriber.LatestMessage;
        xyzPoints = rosReadXYZ(msg); % 提取XYZ点云

        % 计算每个点的距离和角度
        distances = sqrt(xyzPoints(:, 1).^2 + xyzPoints(:, 2).^2 + xyzPoints(:, 3).^2);
        angles = atan2d(xyzPoints(:, 2), xyzPoints(:, 1));

        % 过滤范围和角度
        validIndices = (distances >= minRange) & (distances <= maxRange) & ...
                       (angles >= minAngle) & (angles <= maxAngle);
        filteredPoints = xyzPoints(validIndices, :);

        % 显示过滤后的点云
        if size(filteredPoints, 1) > 0
            % 创建点云对象
            pc = pointCloud(filteredPoints);
            
            % 3D点云显示
            figure(1);
            pcshow(pc, 'VerticalAxis', 'Z', 'VerticalAxisDir', 'Up');
            title(['3D点云 | 距离: ', num2str(minRange), '-', num2str(maxRange), ' 米']);
            xlabel('X'); ylabel('Y'); zlabel('Z');
            drawnow;

            % 初始化障碍物点云
            obstaclePoints = [];

            % 聚类点云
            [labels, numClusters] = pcsegdist(pc, clusterDistanceThreshold);
            
            % 遍历所有聚类，筛选障碍物
            if numClusters > 0
                for i = 1:numClusters
                    % 提取当前聚类的点
                    clusterIndices = (labels == i);
                    clusterPoints = pc.Location(clusterIndices, :);

                    % 计算包围盒大小 (Bounding Box)
                    minVals = min(clusterPoints, [], 1);
                    maxVals = max(clusterPoints, [], 1);
                    clusterSize = maxVals - minVals;

                    % 设定障碍物的识别标准
                    minClusterPoints = 10;  % 最小点数 (小于此值视为噪声)
                    minClusterSize = 0.1;   % 最小障碍物尺寸 (米)

                    % 判断是否符合障碍物标准
                    if (size(clusterPoints, 1) >= minClusterPoints) && all(clusterSize >= minClusterSize)
                        obstaclePoints = [obstaclePoints; clusterPoints]; % 记录障碍物点云
                    end
                end
            end
            
            % 可视化聚类结果
            figure(5);
            pcshow(pc.Location, labels);
            colormap(hsv(numClusters));
            title(['聚类结果 | 聚类数量: ', num2str(numClusters)]);
            xlabel('X (米)'); ylabel('Y (米)'); zlabel('Z (米)');
            drawnow;

            % 可视化障碍物点云
            figure(6);
            clf;
            if ~isempty(obstaclePoints)
                obstaclePc = pointCloud(obstaclePoints);
                pcshow(obstaclePc, 'VerticalAxis', 'Z', 'VerticalAxisDir', 'Up');
                title(['障碍物点云 | 识别数量: ', num2str(size(obstaclePoints, 1))]);
                xlabel('X (米)'); ylabel('Y (米)'); zlabel('Z (米)');
            else
                title('未检测到障碍物');
            end
            drawnow;

            % 仅使用障碍物点云创建占据图
            if exist('obstaclePoints', 'var') && ~isempty(obstaclePoints)
                xEdges = xLimits(1):resolution:xLimits(2);
                yEdges = yLimits(1):resolution:yLimits(2);

                % 使用障碍物点云生成占据图
                occupancyMap = histcounts2(obstaclePoints(:, 1), obstaclePoints(:, 2), xEdges, yEdges);

                % 可视化占据图
                figure(3);
                clf;
                imagesc(xEdges, yEdges, occupancyMap', 'AlphaData', ~isnan(occupancyMap'));
                axis xy;
                colormap(flipud(gray));
                colorbar;
                set(gca, 'Color', 'w');
                caxis([minDensityThreshold maxDensityThreshold]);
                title('基于障碍物的占据图 (Occupancy Map)');
                xlabel('X (米)'); ylabel('Y (米)');
                drawnow;
            else
                disp("未检测到障碍物");
            end

            % 基于占据图生成二值矩阵
            binaryMatrix = occupancyMap > 0;

            % 可视化二值矩阵
            figure(4);
            imagesc(xEdges, yEdges, binaryMatrix', 'AlphaData', ~isnan(binaryMatrix'));
            axis xy;
            colormap(flipud(gray));
            set(gca, 'Color', 'w');
            title('二值矩阵 (Binary Matrix)');
            xlabel('X (米)'); ylabel('Y (米)');
            drawnow;

        end
    end
end

disp("点云处理完成！");
