%% 清理环境
clear; clc; close all;

%================= ROS2节点与订阅者 =================
% 创建 ROS 2 节点，并订阅 /scan 话题 (传感器消息类型 sensor_msgs/LaserScan)
node = ros2node("/matlab_node");
scanSubscriber = ros2subscriber(node, "/scan", "sensor_msgs/LaserScan");

%================= 参数设置 =================
minRange = 0.5;              % 最小距离 (米)
maxRange = 5.0;              % 最大距离 (米)

minAngleDeg = -180;          % 最小角度 (度)
maxAngleDeg = 180;           % 最大角度 (度)

% 占据图分辨率和范围
resolution = 0.1;
xLimits = [-5, 5];           % X 方向范围(米)
yLimits = [-5, 5];           % Y 方向范围(米)

% 聚类距离阈值
clusterDistanceThreshold = 0.2;  % 2D 点之间大于 0.2m 划分到不同簇
minClusterPoints = 10;           % 最少多少点才认为是有效聚类
minClusterSize = 0.1;            % 最小障碍物尺寸(米)

% 占据图可视化时使用的阈值
minDensityThreshold = 10;
maxDensityThreshold = 50;

disp("等待接收 2D 激光雷达数据...");

%================= 主循环: 运行120秒 =================
tic;
while toc < 120
    if ~isempty(scanSubscriber.LatestMessage)
        %---------- 读取最新 LaserScan ----------
        scanMsg = scanSubscriber.LatestMessage;
        
        % LaserScan 的实际字段名(ROS 2): angle_min, angle_max, angle_increment, ranges (小写下划线)
        angleMin = double(scanMsg.angle_min);        
        angleMax = double(scanMsg.angle_max);        
        angleInc = double(scanMsg.angle_increment);  
        ranges   = double(scanMsg.ranges);          
        
        % 计算该帧激光共有多少束
        numPoints = length(ranges);
        
        %---------- 构建角度向量(弧度) ----------
        % anglesRad: 从 angle_min 以 angle_increment 递增到 (angle_min + (numPoints-1)*angleInc)
        anglesRad = (angleMin : angleInc : (angleMin + (numPoints-1)*angleInc))';
        
        %---------- 将极坐标 (ranges, angles) 转为 2D 笛卡尔 (x,y) ----------
        x = ranges .* cos(anglesRad);
        y = ranges .* sin(anglesRad);
        z = zeros(size(x));  % 对于2D激光，Z=0
        
        %---------- 去除无效 (Inf/NaN) 或距离为 0 的点 ----------
        validMask = isfinite(ranges) & (ranges > 0);
        x = x(validMask);
        y = y(validMask);
        z = z(validMask);
        anglesRad = anglesRad(validMask);
        ranges    = ranges(validMask);
        
        %---------- 按距离和角度过滤 ----------
        anglesDeg = rad2deg(anglesRad);
        validIndices = (ranges >= minRange) & (ranges <= maxRange) & ...
                       (anglesDeg >= minAngleDeg) & (anglesDeg <= maxAngleDeg);
        filteredX = x(validIndices);
        filteredY = y(validIndices);
        filteredZ = z(validIndices);
        
        if ~isempty(filteredX)
            %% 构建点云对象 (MATLAB里pcshow需要 Nx3)
            pc = pointCloud([filteredX, filteredY, filteredZ]);
            
            %================= 1) 显示过滤后的点云 =================
            figure(1);
            pcshow(pc, 'VerticalAxis', 'Z', 'VerticalAxisDir', 'Up');
            title(['2D 激光点云 (Z=0) | 距离: ', num2str(minRange), ' ~ ', num2str(maxRange), ' m']);
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            drawnow;
            
            %================= 2) 基于距离的聚类 =================
            [labels, numClusters] = pcsegdist(pc, clusterDistanceThreshold);
            
            %----------------- 聚类可视化 -----------------
            figure(2);
            pcshow(pc.Location, labels);
            colormap(hsv(numClusters));  % 不同簇用不同颜色
            title(['聚类结果 | 聚类数量: ', num2str(numClusters)]);
            xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
            drawnow;
            
            %----------------- 根据聚类提取障碍物 -----------------
            obstaclePoints = [];
            if numClusters > 0
                for i = 1:numClusters
                    clusterIdx = (labels == i);
                    clusterPts = pc.Location(clusterIdx, :);
                    
                    % 计算该聚类的包围盒大小
                    minVals = min(clusterPts, [], 1);
                    maxVals = max(clusterPts, [], 1);
                    clusterSize = maxVals - minVals;
                    
                    % 判断是否符合障碍物标准
                    if (size(clusterPts, 1) >= minClusterPoints) && all(clusterSize >= minClusterSize)
                        obstaclePoints = [obstaclePoints; clusterPts];
                    end
                end
            end

            %================= 4) 构建 Occupancy Map (栅格) =================
            if ~isempty(obstaclePoints)
                xEdges = xLimits(1):resolution:xLimits(2);
                yEdges = yLimits(1):resolution:yLimits(2);
                
                % 利用 histcounts2 对 X,Y 做统计, 生成占据图
                occupancyMap = histcounts2(obstaclePoints(:,1), ...
                                           obstaclePoints(:,2), ...
                                           xEdges, yEdges);
                
                % 显示占据图
                figure(4); clf;
                imagesc(xEdges, yEdges, occupancyMap', 'AlphaData', ~isnan(occupancyMap'));
                axis xy;       % 让 X,Y 轴顺着正常方向
                colormap(flipud(gray));
                colorbar;
                set(gca,'Color','w');
                caxis([minDensityThreshold maxDensityThreshold]);
                title('基于障碍物的占据图 (Occupancy Map)');
                xlabel('X (m)'); ylabel('Y (m)');
                drawnow;
            else
                % 如果没有障碍物，occupancyMap 可以设为空或0
                disp("未检测到障碍物，occupancyMap 未更新");
                occupancyMap = [];
            end
            
            %================= 5) 二值矩阵 =================
            if ~isempty(obstaclePoints)
                binaryMatrix = occupancyMap > 0;
                
                figure(5);
                imagesc(xEdges, yEdges, binaryMatrix', 'AlphaData', ~isnan(binaryMatrix'));
                axis xy;
                colormap(flipud(gray));
                set(gca, 'Color','w');
                title('二值矩阵 (Binary Matrix)');
                xlabel('X (m)'); ylabel('Y (m)');
                drawnow;
            end
        end
    end
end

disp("2D 激光雷达数据处理完成！");
