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

disp("等待接收点云数据...");

% 主循环：接收并处理点云数据
tic;
while toc < 60 % 持续运行60秒，用户可调整
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
            % 创建点云对象并显示3D点云
            pc = pointCloud(filteredPoints);
            figure(1);
            pcshow(pc, 'VerticalAxis', 'Z', 'VerticalAxisDir', 'Up');
            title(['3D点云限制 | 距离: ', num2str(minRange), '-', num2str(maxRange), ' 米, 角度: ', ...
                   num2str(minAngle), '° 到 ', num2str(maxAngle), '°']);
            xlabel('X'); ylabel('Y'); zlabel('Z');
            drawnow;

            % 2D平面显示 (投影到XY平面)
            figure(2);
            plot(filteredPoints(:, 1), filteredPoints(:, 2), '.');
            axis equal;
            title(['2D点云投影 | 距离: ', num2str(minRange), '-', num2str(maxRange), ' 米, 角度: ', ...
                   num2str(minAngle), '° 到 ', num2str(maxAngle), '°']);
            xlabel('X (米)'); ylabel('Y (米)');
            grid on;
            drawnow;

            % 创建栅格地图
            xEdges = xLimits(1):resolution:xLimits(2);
            yEdges = yLimits(1):resolution:yLimits(2);
            occupancyMap = histcounts2(filteredPoints(:, 1), filteredPoints(:, 2), xEdges, yEdges);

            % 可视化占据图
            figure(3);
            imagesc(xEdges, yEdges, occupancyMap', 'AlphaData', ~isnan(occupancyMap'));
            axis xy;
            colormap(flipud(gray));
            colorbar;
            set(gca, 'Color', 'w');
            caxis([minDensityThreshold maxDensityThreshold]);
            title('占据图 (Occupancy Map)');
            xlabel('X (米)'); ylabel('Y (米)');
            drawnow;

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

disp("点云显示完成！");