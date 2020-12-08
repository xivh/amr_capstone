filename = "C:\Users\student\Desktop\Katie\Capstone\horizon tests bag files\horizon tests bag files\complex path 2.csv";
complexpaths1 = csvread(filename);

filename2 = "C:\Users\student\Desktop\Katie\Capstone\horizon tests bag files\horizon tests bag files\smooth_path.csv";
smooth_path = csvread(filename2);

filename3 = "C:\Users\student\Desktop\Katie\Capstone\horizon tests bag files\horizon tests bag files\12_7_meeting_data";
debug_data = xlsread(filename3, 1);

x = debug_data(:,1);
y = debug_data(:,2);
yaw_state = rad2deg(debug_data(:,3)); %degrees
lookAhead_idx = debug_data(:,4);
localGoalx = debug_data(:,5);
localGoaly = debug_data(:,6);
vel = debug_data(:,7);
angle = abs(rad2deg(debug_data(:,8))); % absolute value of degrees
ang_vel = debug_data(:,9);
abs_yaw = rad2deg(debug_data(:,10)); % degrees
atantwo = rad2deg(debug_data(:,11));

figure(1)
title('Path')
hold on
plot(x,y,'-','LineWidth',3, 'Color', 'c')
hold on
plot(localGoalx,localGoaly,'*','MarkerSize', 2, 'Color', 'r')
hold on
plot(complexpaths1(:,1), complexpaths1(:,2),'-','LineWidth',0.5, 'Color', 'k')
hold on
legend("Robot Path", "Look Ahead Points") %, "Given Path")
grid on

figure(2)
title('Linear Velocity')
hold on
z = zeros(size(x))';
col = vel;  % This is the color, vary with x in this case.
surface([x';x'],[y';y'],[z;z],[col';col'],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',4);
ylabel("Linear Velocity (m/s)");
colorbar

% figure(3)
% title('Yaw Over Path')
% hold on
% z = zeros(size(x))';
% col = yaw_state;  % This is the color, vary with x in this case.
% surface([x';x'],[y';y'],[z;z],[col';col'],...
%         'facecol','no',...
%         'edgecol','interp',...
%         'linew',4);
% colorbar

% figure(4)
% title('Angle Over Path')
% hold on
% z = zeros(size(x))';
% col = angle;  % This is the color, vary with x in this case.
% surface([x';x'],[y';y'],[z;z],[col';col'],...
%         'facecol','no',...
%         'edgecol','interp',...
%         'linew',4);
% colorbar

% figure(5)
% title('Angular Velocity Over Path')
% hold on
% z = zeros(size(x))';
% col = ang_vel;  % This is the color, vary with x in this case.
% surface([x';x'],[y';y'],[z;z],[col';col'],...
%         'facecol','no',...
%         'edgecol','interp',...
%         'linew',4);
% colorbar