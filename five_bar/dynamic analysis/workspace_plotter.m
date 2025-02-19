% This plots the workspace points from initial_pts, classifying them by
% working configuration using pastel colors and transparency.
%
% Author: Juan Pablo Reyes
clearvars
close all

pts = table2array(parquetread("initial_pts.parquet"));

% Extract coordinates
x = pts(:,5);
y = pts(:,6);

% Compute conditions
cond1 = (pts(:,2) - pts(:,1) > 0) & (pts(:,3) - pts(:,4) > 0);  % Case 1
cond2 = (pts(:,2) - pts(:,1) > 0) & (pts(:,3) - pts(:,4) <= 0); % Case 2
cond3 = (pts(:,2) - pts(:,1) <= 0) & (pts(:,3) - pts(:,4) > 0); % Case 3
cond4 = (pts(:,2) - pts(:,1) <= 0) & (pts(:,3) - pts(:,4) <= 0);% Case 4

% Define pastel colors (softer)
color1 = [1 0.5 0.5];  % Soft Red
color2 = [0.5 1 0.5];  % Soft Green
color3 = [0.5 0.5 1];  % Soft Blue
color4 = [1 1 0.6];    % Soft Yellow

color5 = [1 1 1];    % Soft Yellow

% Create figure
figure;

% Subplot 1 - Case 1 (Soft Red)
subplot(2,2,1);
scatter(x(cond1), y(cond1), 10, color1, 'filled', 'MarkerFaceAlpha', 0.3);
title('Case 1: (+,+)', 'FontSize', 14, 'FontWeight', 'Bold', 'Interpreter', 'latex');
xlabel('$X$', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('$Y$', 'FontSize', 12, 'Interpreter', 'latex');
xlim([-0.4 0.4]); ylim([-0.4 0.4]);
axis equal; grid on;

% Subplot 2 - Case 2 (Soft Green)
subplot(2,2,2);
scatter(x(cond2), y(cond2), 10, color2, 'filled', 'MarkerFaceAlpha', 0.3);
title('Case 2: (+,-)', 'FontSize', 14, 'FontWeight', 'Bold', 'Interpreter', 'latex');
xlabel('$X$', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('$Y$', 'FontSize', 12, 'Interpreter', 'latex');
xlim([-0.4 0.4]); ylim([-0.4 0.4]);
axis equal; grid on;

% Subplot 3 - Case 3 (Soft Blue)
subplot(2,2,3);
scatter(x(cond3), y(cond3), 10, color3, 'filled', 'MarkerFaceAlpha', 0.3);
title('Case 3: (-,+)', 'FontSize', 14, 'FontWeight', 'Bold', 'Interpreter', 'latex');
xlabel('$X$', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('$Y$', 'FontSize', 12, 'Interpreter', 'latex');
xlim([-0.4 0.4]); ylim([-0.4 0.4]);
axis equal; grid on;

% Subplot 4 - Case 4 (Soft Yellow)
subplot(2,2,4);
scatter(x(cond4), y(cond4), 10, color4, 'filled', 'MarkerFaceAlpha', 0.3);
title('Case 4: (-,-)', 'FontSize', 14, 'FontWeight', 'Bold', 'Interpreter', 'latex');
xlabel('$X$', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('$Y$', 'FontSize', 12, 'Interpreter', 'latex');
xlim([-0.4 0.4]); ylim([-0.4 0.4]);
axis equal; grid on;

% Global title
sgtitle('Workspace for Available Configurations', 'FontSize', 16, 'FontWeight', 'Bold', 'Interpreter', 'latex');

% Save as high-resolution image
set(gcf, 'Color', 'w'); % White background

figure;
ax = axes;
set(gca, 'Color', 'none');  % Set axes background transparent
set(gcf, 'Color', 'none');  % Set figure background transparent
scatter(x, y, 10, color5, 'filled', 'MarkerFaceAlpha', 0.3);
title('Case 4: (-,-)', 'FontSize', 14, 'FontWeight', 'Bold', 'Interpreter', 'latex');
xlabel('$X$', 'FontSize', 12, 'Interpreter', 'latex');
ylabel('$Y$', 'FontSize', 12, 'Interpreter', 'latex');
xlim([-0.4 0.4]); ylim([-0.4 0.4]);
axis equal; grid on;

