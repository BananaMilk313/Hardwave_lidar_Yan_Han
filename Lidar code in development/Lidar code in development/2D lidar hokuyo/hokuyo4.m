function lidarRealtime3View
    % 说明：
    % 1) 使用 ROS2 订阅 /scan 数据；
    % 2) 仅保留「过滤后的点云」、「聚类结果」、「俯视图」三部分可视化；
    % 3) 采用与 laneDetectionRealtime_dynamicThresholdWithOccupancyMap 类似的单窗口 + subplot + clf 刷新方式；
    % 4) 在 while true（或 while isvalid(hFig)）循环内持续更新，需要手动关闭窗口或 Ctrl+C 退出。

    %% ========== 1. 清理环境并初始化 ROS2 节点、订阅者 ==========
    clear; clc; close all;

    node = ros2node("/matlab_node");
    scanSubscriber = ros2subscriber(node, ...
        "/scan", "sensor_msgs/LaserScan");

    %% ========== 2. 参数设置 ==========
    minRange = 0.5;     % 最小距离 (米)
    maxRange = 5.0;     % 最大距离 (米)
    minAngleDeg = -90; % 最小角度 (度)
    maxAngleDeg = 90;  % 最大角度 (度)

    % 聚类距离阈值
    clusterDistanceThreshold = 0.1;  % 同一簇内相邻点间距不超过 0.2m
    minClusterPoints = 10;          % 最少点数才认为是有效簇
    minClusterSize = 0.1;           % 簇在 x/y 方向的最小“宽度”，小于此不认为是障碍物

    % 俯视图范围
    xLimits = [-5, 5];   
    yLimits = [-5, 5];
    gridSpacing = 0.01;  % 网格间隔(米) => 1 cm

    disp("等待接收 2D 激光雷达数据...");

    %% ========== 3. 创建单个 figure，内部使用 subplot 来显示不同结果 ==========
    hFig = figure('Name','Real-time Laser Scan Processing','NumberTitle','off',...
                  'Units','normalized','Position',[0.05 0.05 0.9 0.8]);

    %% ========== 4. 主循环：只要窗口存在，就持续读取 /scan 并刷新显示 ==========
    while isvalid(hFig)
        frameTimer = tic;  % 统计每次循环的处理耗时

        % 如果当前有新的 scan 消息
        if ~isempty(scanSubscriber.LatestMessage)
            scanMsg = scanSubscriber.LatestMessage;

            %---------------- 读取并处理 LaserScan ----------------
            angleMin = double(scanMsg.angle_min);
            angleMax = double(scanMsg.angle_max);
            angleInc = double(scanMsg.angle_increment);
            ranges   = double(scanMsg.ranges);
            numPoints = length(ranges);

            anglesRad = (angleMin : angleInc : (angleMin + (numPoints-1)*angleInc))';
            x = ranges .* cos(anglesRad);
            y = ranges .* sin(anglesRad);
            z = zeros(size(x));

            % 去除无效/距离≤0 的点
            validMask = isfinite(ranges) & (ranges > 0);
            x = x(validMask);
            y = y(validMask);
            z = z(validMask);
            anglesRad = anglesRad(validMask);
            ranges    = ranges(validMask);

            % 根据距离和角度阈值进一步过滤
            anglesDeg = rad2deg(anglesRad);
            validIndices = (ranges >= minRange) & (ranges <= maxRange) & ...
                           (anglesDeg >= minAngleDeg) & (anglesDeg <= maxAngleDeg);
            filteredX = x(validIndices);
            filteredY = y(validIndices);
            filteredZ = z(validIndices);

            % 如果过滤后仍有点，则做可视化
            if ~isempty(filteredX)
                pc = pointCloud([filteredX, filteredY, filteredZ]);

                %================= (1) 过滤后的激光点云 =================
                % 清理并切换到 figure+subplot，然后渲染点云
                clf(hFig);  % 每帧都清空整个 figure，但不关闭窗口
                subplot(2,2,1);
                pcshow(pc, 'VerticalAxis','Z','VerticalAxisDir','Up');
                title(sprintf('1) 过滤后的点云\nRange: %.1f ~ %.1f m', ...
                              minRange, maxRange));
                xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

                %================= (2) 基于距离的聚类 =================
                [labels, numClusters] = pcsegdist(pc, clusterDistanceThreshold);
                subplot(2,2,2);
                pcshow(pc.Location, labels);
                if numClusters > 0
                    colormap(hsv(numClusters));  % 给不同簇上色
                end
                title(sprintf('2) 聚类结果, 簇数 = %d', numClusters));
                xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');

                % 可选：提取障碍物点 (不做单独显示，这里仅演示)
                obstaclePoints = [];
                for i = 1 : numClusters
                    clusterIdx = (labels == i);
                    clusterPts = pc.Location(clusterIdx, :);

                    minVals = min(clusterPts, [], 1);
                    maxVals = max(clusterPts, [], 1);
                    clusterSize = maxVals - minVals;

                    % 判断是否符合障碍物的要求
                    if (size(clusterPts,1) >= minClusterPoints) && all(clusterSize >= minClusterSize)
                        obstaclePoints = [obstaclePoints; clusterPts];
                    end
                end

%%(3) 俯视图
                subplot(2,2,3);
                plot(filteredX, filteredY, '.');
                hold on; axis equal;
                xlim(xLimits); ylim(yLimits);
                grid on;
                set(gca, 'XTick', xLimits(1):gridSpacing:xLimits(2), ...
                         'YTick', yLimits(1):gridSpacing:yLimits(2));
                title('3) 俯视图 (Top-Down View)');
                xlabel('X (m)'); ylabel('Y (m)');
                hold off;

%% (4) 文本信息
                subplot(2,2,4);
                axis off;
                procTime = toc(frameTimer);
                txt = sprintf(['激光帧处理耗时：%.3f s\n' ...
                               '过滤后点数：%d\n' ...
                               '聚类数量：%d'], ...
                               procTime, length(filteredX), numClusters);
                text(0.1, 0.5, txt, 'FontSize', 12);
            end
        end

        drawnow;
        pause(0.03);  % 适当让出 CPU，避免过度占用
    end

    disp("程序结束或窗口被关闭。");
end
