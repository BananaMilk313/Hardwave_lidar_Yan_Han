% 清理环境
clear; clc; close all;

% 创建ROS 2节点并订阅点云话题
node = ros2node("/matlab_node"); % 创建ROS 2节点
pointsSubscriber = ros2subscriber(node, "/velodyne_points", "sensor_msgs/PointCloud2");

% 设置范围限制
minRange = 0.5; % 最小范围 (米)
maxRange = 5; % 最大范围 (米)

% 设置角度范围限制 (以度为单位)
minAngle = -180; % 最小角度
maxAngle = 180;  % 最大角度

disp("等待接收点云数据...");

% 主循环：接收点云并限制范围和角度
tic;
while toc < 10 % 持续运行10秒，仅显示一帧点云
    % 检查是否有新消息
    if ~isempty(pointsSubscriber.LatestMessage)
        % 获取最新点云消息
        msg = pointsSubscriber.LatestMessage;
        xyzPoints = rosReadXYZ(msg); % 提取XYZ点云

        % 计算每个点的距离
        distances = sqrt(xyzPoints(:, 1).^2 + xyzPoints(:, 2).^2 + xyzPoints(:, 3).^2);

        % 计算每个点的角度
        angles = atan2d(xyzPoints(:, 2), xyzPoints(:, 1)); % 计算角度 (度)

        % 过滤范围和角度
        validIndices = (distances >= minRange) & (distances <= maxRange) & ...
                       (angles >= minAngle) & (angles <= maxAngle);
        filteredPoints = xyzPoints(validIndices, :);

        % 显示过滤后的点云
        if size(filteredPoints, 1) > 0
            % 创建点云对象并显示3D点云
            pc = pointCloud(filteredPoints); % 创建点云对象
            figure(1);
            pcshow(pc, 'VerticalAxis', 'Z', 'VerticalAxisDir', 'Up');
            title(['3D点云限制 | 距离: ', num2str(minRange), '-', num2str(maxRange), ' 米, 角度: ', ...
                   num2str(minAngle), '° 到 ', num2str(maxAngle), '°']);
            xlabel('X'); ylabel('Y'); zlabel('Z');
            drawnow;

            % 2D平面显示 (投影到XY平面)
            figure(2);
            plot(filteredPoints(:, 1), filteredPoints(:, 2), '.'); % 仅绘制XY坐标
            axis equal;
            title(['2D点云投影 | 距离: ', num2str(minRange), '-', num2str(maxRange), ' 米, 角度: ', ...
                   num2str(minAngle), '° 到 ', num2str(maxAngle), '°']);
            xlabel('X (米)'); ylabel('Y (米)');
            grid on;
            drawnow;

            % 生成占据图 (Occupancy Map)
            resolution = 0.1; % 栅格分辨率 (米/格)
            xLimits = [min(filteredPoints(:, 1)), max(filteredPoints(:, 1))];
            yLimits = [min(filteredPoints(:, 2)), max(filteredPoints(:, 2))];

            % 创建栅格地图
            xEdges = xLimits(1):resolution:xLimits(2);
            yEdges = yLimits(1):resolution:yLimits(2);
            occupancyMap = histcounts2(filteredPoints(:, 1), filteredPoints(:, 2), xEdges, yEdges);

            % 可视化占据图
            figure(3);
            imagesc(xEdges, yEdges, occupancyMap', 'AlphaData', ~isnan(occupancyMap')); % 背景透明
            axis xy;
            colormap(flipud(gray)); % 使用反转的灰度颜色图
            colorbar;
            set(gca, 'Color', 'w'); % 设置背景为白色
            title('占据图 (Occupancy Map)');
            xlabel('X (米)'); ylabel('Y (米)');
            drawnow;
        end
        break; % 只加载一帧后退出
    end
end

disp("点云角度和范围限制完成！");
