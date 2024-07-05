%% IiA Auto Scan2BIM
clear;clc;
dataset = readmatrix('road_edge_label.csv');

data = dataset(:,1:3);
labels = dataset(:,4);

% colors = [
%     1 0 0;  % Red label 1
%     0 1 0;  % Green label 2
%     0 0 1;  % Blue label 3
%     1 1 0;  % Yellow label 4
%     0 1 1;  % Cyan label 5
% ];

%%
figure(1)
scatter3(data(:,1), data(:,2), data(:,3)); 

%%
figure(2)
uniqueLabels = unique(labels);  
numLabels = length(uniqueLabels);  
colors = lines(numLabels);

hold on;  
for i = 1:numLabels
    idx = labels == uniqueLabels(i);
    scatter3(data(idx, 1), data(idx, 2), data(idx, 3), 36, colors(i,:), 'filled');
end
hold off;
xlabel('X Axis');  ylabel('Y Axis');  zlabel('Z Axis'); 
title('3D Data Plot');  
legend(arrayfun(@num2str, uniqueLabels, 'UniformOutput', false)); 

%%
figure(3)
    idx = labels == 4;
    scatter3(data(idx, 1), data(idx, 2), data(idx, 3), 36, colors(i,:), 'filled');
    xlabel('X Axis'); 
    ylabel('Y Axis');  
    zlabel('Z Axis'); 
    title(['3D Scatter Plot for Label ', num2str(4)]);  
    grid on; 

figure(4)    
plot(data(idx, 1), data(idx, 2), '.');    
xlabel('X axis');       
ylabel('Y axis');      
title('2D Points');  
grid on;            

%%
x = data(idx, 1); 
y = data(idx, 2); 
z = data(idx, 3);

%%
% 最最重要的是确定两个参数  iter  threshold    
% 目前尝试的 iter=2000 和 threshold=5 会有比较好的结果

[model1, inlierIdx1] = ransac(x, y, 3000, 3.5);

% 索引
originalIndices = (1:length(x))';  % 保存索引

% Remove Inliers
x2 = x(~inlierIdx1);
y2 = y(~inlierIdx1);
indices2 = originalIndices(~inlierIdx1);  

% RANSAC Again
[model2, inlierIdx2] = ransac(x2, y2, 3000, 3.5);

originalInlierIdx1 = originalIndices(inlierIdx1);
originalInlierIdx2 = indices2(inlierIdx2);

figure(5)
hold on;
scatter(x, y, 'b.');
% scatter(x(inlierIdx1), y(inlierIdx1), 'ro');
% scatter(x2(inlierIdx2), y2(inlierIdx2), 'go');
scatter(x(originalInlierIdx1), y(originalInlierIdx1), 'r.');
scatter(x(originalInlierIdx2), y(originalInlierIdx2), 'g.');
xfit = linspace(min(x), max(x), 100);
plot(xfit, polyval(model1, xfit), 'r', 'LineWidth', 2);
plot(xfit, polyval(model2, xfit), 'g', 'LineWidth', 2);
legend('Data', 'First Fit', 'Second Fit');
title('RANSAC Curve Fitting');
xlabel('X');
ylabel('Y');
grid on;


%%
x_data = x(originalInlierIdx1);  
y_data = y(originalInlierIdx1);  
z_data = z(originalInlierIdx1);  

% polyfit
p = polyfit(x_data, y_data, 2); 
% 5000点
x_fit = linspace(min(x_data), max(x_data), 5000)';
y_fit = polyval(p, x_fit);

% 插值
F = scatteredInterpolant(x_data, y_data, z_data); 
%F = scatteredInterpolant(x_data, y_data, z_data, 'natural', 'none'); 

z_fit = F(x_fit, y_fit);

% Figure
figure(7)
scatter3(x_data, y_data, z_data, 'filled');
hold on;
plot3(x_fit, y_fit, z_fit, 'r');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Curve Fitting with Z Interpolation');
legend('Original Data', 'Fitted Curve');
grid on;


%% 高斯平均值法 去掉噪音点

x_data = x(originalInlierIdx1);  
y_data = y(originalInlierIdx1);  
z_data = z(originalInlierIdx1);  

x_data1 = x(originalInlierIdx1);  
y_data1 = y(originalInlierIdx1);  
z_data1 = z(originalInlierIdx1);  

% 设定高斯函数的标准差
sigma = 12;  % 这个值可以根据实际情况调整

% 差值阈值数组
thresholds = [3, 2, 1];

% 循环处理每个阈值
for k = 1:length(thresholds)
    threshold = thresholds(k);
    
    % 初始化结果数组
    z_weighted_average = zeros(length(x_data), 1);  % 存储加权平均结果，对每个点计算

    % 计算每个点的加权 z 平均
    for i = 1:length(x_data)
        x_i = x_data(i);
        weights = exp(-(x_data - x_i).^2 / (2 * sigma^2));  % 计算权重
        z_weighted_average(i) = sum(z_data .* weights) / sum(weights);  % 计算加权平均
    end

    % 比较加权平均与原始 z 坐标的差值，删除差值大于当前阈值的点
    difference = abs(z_weighted_average - z_data);  % 计算差值
    indices_to_keep = difference <= threshold;  % 找到差值小于或等于当前阈值的索引

    % 保留差值小于等于当前阈值的点
    x_data = x_data(indices_to_keep);
    y_data = y_data(indices_to_keep);
    z_data = z_data(indices_to_keep);
    
    % 输出每次迭代后的结果数量
    fprintf('Iteration %d: Remaining data points = %d\n', k, length(x_data));

    figure;
    plot(x_data1, z_data1, '.');
    hold on;
    plot(x_data, z_data, 'o');

end

%% Upper/Lower  Boundary 

% 计算 x_data 的范围
x_min = min(x_data);
x_max = max(x_data);

% 初始化结果数组
max_results = [];
min_results = [];
mean_y_results = [];

% 遍历 x_data 从 x_min 到 x_max，每一米为单位
for x_target = x_min:x_max
    % 找到 x_target 前后两米范围内的点
    indices = x_data >= (x_target - 2) & x_data <= (x_target + 2);
    
    % 从这些点中获取 z 坐标的最大值和最小值
    if any(indices)  % 确保至少有一个点在这个范围内
        max_z = max(z_data(indices));
        min_z = min(z_data(indices));
        mean_y = mean(y_data(indices));
    else
        max_z = NaN;  % 如果没有点在范围内，返回 NaN
        min_z = NaN;
        mean_y = NaN;

    end
    
    % 将结果存储到数组
    max_results = [max_results; x_target, max_z];
    min_results = [min_results; x_target, min_z];
    mean_y_results = [mean_y_results; x_target, mean_y];
end

% 分离 x、max_z 和 min_z
x_plot = max_results(:, 1);
max_z_plot = max_results(:, 2);
min_z_plot = min_results(:, 2);
mean_y_plot = mean_y_results(:, 2);

% 绘制原始数据点
figure;
plot(x_plot, max_z_plot, 'bo', 'DisplayName', 'Max Z');
hold on;
plot(x_plot, min_z_plot, 'ro', 'DisplayName', 'Min Z');

% 使用光滑曲线拟合
x_dense = linspace(min(x_plot), max(x_plot), 1000);
max_z_smooth = interp1(x_plot, max_z_plot, x_dense, 'pchip');
min_z_smooth = interp1(x_plot, min_z_plot, x_dense, 'pchip');
mean_y_smooth = interp1(x_plot, mean_y_plot, x_dense, 'pchip');

plot(x_dense, max_z_smooth, 'b-', 'DisplayName', 'Fitted Max Curve');
plot(x_dense, min_z_smooth, 'r-', 'DisplayName', 'Fitted Min Curve');

% 设置图形属性
title('Max and Min Z values over X with Smooth Curves');
xlabel('X coordinate');
ylabel('Z coordinate');
legend('show');
grid on;

% 显示图形
hold off;

figure; % 创建一个新图形窗口
plot3(x_dense, mean_y_smooth, max_z_smooth, 'o-'); % 绘制三维散点图

% 添加标签和标题
xlabel('X coordinate');
ylabel('Y coordinate (Mean Y)');
zlabel('Z coordinate (Max Z)');
title('3D Plot of X vs Mean Y vs Max Z');
grid on; % 添加网格
rotate3d on; % 允许使用鼠标旋转图形

% 可以选择使用不同的颜色和标记样式
% plot3(x_dense, mean_y_smooth, max_z_smooth, 'r.-'); % 红色点和线

% 显示图形
hold off;



%% 使用前面生成的 x_dense, max_z_smooth 和 mean_y_smooth 创建插值函数
zmax_interp_func = @(x_query) interp1(x_dense, max_z_smooth, x_query, 'pchip');
zmin_interp_func = @(x_query) interp1(x_dense, min_z_smooth, x_query, 'pchip');
y_interp_func = @(x_query) interp1(x_dense, mean_y_smooth, x_query, 'pchip');

% 给定一个 x 点
x_query = (min(x_dense)+max(x_dense))/2;  % 你可以根据需要修改这个值

% 检查 x_query 是否在 x_dense 范围内
if x_query < min(x_dense) || x_query > max(x_dense)
    error('Query x value is out of the interpolation range.');
end 

% 使用插值函数获取 z 和 y 坐标
zmax_value = zmax_interp_func(x_query);
zmin_value = zmin_interp_func(x_query);
y_value = y_interp_func(x_query);

% 输出结果
fprintf('The estimated z max value at x = %f is %f.\n', x_query, zmax_value);
fprintf('The estimated z min value at x = %f is %f.\n', x_query, zmin_value);
fprintf('The estimated y value at x = %f is %f.\n', x_query, y_value);

%%
% 使用前面生成的 x_dense, max_z_smooth 和 mean_y_smooth 创建插值函数
zmax_interp_func = @(x_query) interp1(x_dense, max_z_smooth, x_query, 'pchip');
zmin_interp_func = @(x_query) interp1(x_dense, min_z_smooth, x_query, 'pchip');
y_interp_func = @(x_query) interp1(x_dense, mean_y_smooth, x_query, 'pchip');

% 创建等间隔的 x 值
x_values = linspace(x_min, x_max, 1000);

% 初始化 y 和 z 坐标数组
y_values = zeros(1, 1000);
zmax_values = zeros(1, 1000);
zmin_values = zeros(1, 1000);

% 计算对应的 y 和 z 坐标
for i = 1:1000
    y_values(i) = y_interp_func(x_values(i));
    zmax_values(i) = zmax_interp_func(x_values(i));
    zmin_values(i) = zmin_interp_func(x_values(i));
end

% 绘制图形
figure;
plot3(x_values, y_values, zmax_values, 'LineWidth', 2);hold on;
plot3(x_values, y_values, zmin_values, 'LineWidth', 2);
grid on;
title('3D Plot of Interpolated XYZ Coordinates');
xlabel('X');
ylabel('Y');
zlabel('Z');





% %% DBSCAN
% x_data = x(originalInlierIdx1);  
% y_data = y(originalInlierIdx1);  
% z_data = z(originalInlierIdx1);  
% 
% 
% % 合并XYZ数据为一个N-by-3矩阵，N是点的数量
% data = [x_data(:), y_data(:), z_data(:)];
% % 设定DBSCAN参数
% epsilon = 3;  % ε，搜索半径
% MinPts = 12;     % MinPts
% % DBSCAN算法
% labels = dbscan(data, epsilon, MinPts);
% % labels  -1噪声点
% 
% % 过滤出有效点
% filtered_data = data(labels == 1, :);
% % 可选：分别获取过滤后的x, y, z数据
% x_filtered = filtered_data(:, 1);
% y_filtered = filtered_data(:, 2);
% z_filtered = filtered_data(:, 3);
% 
% figure(11)
% scatter3(x_data, y_data, z_data, '.'); hold on;
% scatter3(x_filtered, y_filtered, z_filtered, 'o', 'filled'); hold off;
% legend('Original data', 'Filtered data');
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('DBSCAN Filtering');
% grid on;
% 
% % 5000点
% % polyfit
% p = polyfit(x_data, y_data, 2); 
% x_fit = linspace(min(x_filtered), max(y_filtered), 100)';
% y_fit = polyval(p, x_fit);
% 
% % 插值
% F = scatteredInterpolant(x_filtered, y_filtered, z_filtered); 
% z_fit = F(x_fit, y_fit);
% 
% % Figure
% figure;
% scatter3(x_filtered, y_filtered, z_filtered, 'filled');
% hold on;
% plot3(x_fit, y_fit, z_fit, '.'); hold off;
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% title('Curve Fitting with Z Interpolation');
% legend('Original Data', 'Fitted Curve');
% grid on;
% 


%%
% multiRansac
% 6 curves
numLines = 6;
[models, allInliers] = multiRansac(x, y, numLines, 2000, 5);

colors = ['r', 'g', 'b', 'k', 'm', 'c'];  
figure(6)
hold on;
scatter(x, y, 'o');
for i = 1:numLines
    inlierX = x(allInliers{i});
    inlierY = y(allInliers{i});
    xfit = linspace(min(inlierX), max(inlierX), 100);
    plot(xfit, polyval(models{i}, xfit), colors(i), 'LineWidth', 2);
end
title('RANSAC Multiple Curve Fitting');
xlabel('X');
ylabel('Y');
grid on;


%%
function [model, inlierIdx] = ransac(x, y, iter, threshold)
    numPts = length(x);
    bestInNum = 0;  
    for i = 1:iter
        sampleIdx = randperm(numPts, 3);
        xSample = x(sampleIdx);
        ySample = y(sampleIdx);

        p = polyfit(xSample, ySample, 2);

        yCalc = polyval(p, x);
        errors = abs(y - yCalc);

        inlierIdxTmp = errors < threshold;
        inlierNum = sum(inlierIdxTmp);

        if inlierNum > bestInNum
            bestInNum = inlierNum;
            model = p;
            inlierIdx = inlierIdxTmp;
        end
    end
end


function [models, allInliers] = multiRansac(x, y, numLines, iter, threshold)
    models = cell(numLines, 1);
    allInliers = cell(numLines, 1);
    currentX = x;
    currentY = y;

    for n = 1:numLines
        [model, inlierIdx] = ransac(currentX, currentY, iter, threshold);
        models{n} = model;
        allInliers{n} = inlierIdx;

        currentX = currentX(~inlierIdx);
        currentY = currentY(~inlierIdx);
    end
end
