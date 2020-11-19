filename = "C:\Users\student\Desktop\Katie\Capstone\horizon tests bag files\horizon tests bag files\complex path 1.csv";
complexpaths1 = csvread(filename);

filename3 = "C:\Users\student\Desktop\Katie\Capstone\horizon tests bag files\horizon tests bag files\debug7.csv";
debug_data = csvread(filename3);


x = debug_data(:,1);
y = debug_data(:,2);
yaw_state = rad2deg(debug_data(:,3));
yaw_odom = rad2deg(debug_data(:,4));
quat_state = debug_data(:,5);
quat_odom = debug_data(:,6);
closest_idx = debug_data(:,7);
localGoalx = debug_data(:,8);
localGoaly = debug_data(:,9);
goal_angle = rad2deg(debug_data(:,10));
vel = debug_data(:,11);
ang_vel = goal_angle-yaw_state;
pathx = complexpaths1(1:789,1);
pathy = complexpaths1(1:789,2);
%deviation = sqrt((pathx-x).^2+(pathy-y).^2);

figure(1)
title('Compare Yaws')
hold on
plot(yaw_state,'-','LineWidth',3, 'Color', 'c')
hold on
plot(yaw_odom,'-','LineWidth',1, 'Color', 'r')
hold on
legend("Model States", "Odometry")
grid on

figure(2)
title('Compare Quats')
hold on
plot(quat_state,'-','LineWidth',3, 'Color', 'c')
hold on
plot(quat_odom,'-','LineWidth',1, 'Color', 'r')
hold on
legend("Model States", "Odometry")
grid on

figure(3)
title('Path')
hold on
plot(x,y,'-','LineWidth',3, 'Color', 'c')
hold on
plot(localGoalx,localGoaly,'*','MarkerSize', 2, 'Color', 'r')
hold on
plot(complexpaths1(:,1), complexpaths1(:,2),'-','LineWidth',0.5, 'Color', 'k')
hold on
legend("Robot Path", "Look Ahead Points", "Given Path")
grid on

figure(4)
title('Index Over Path')
hold on
z = zeros(size(x))';
col = closest_idx;  % This is the color, vary with x in this case.
surface([x';x'],[y';y'],[z;z],[col';col'],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',4);
ylabel("Linear Velocity (m/s)");
colorbar

figure(5)
title('Index')
hold on
plot(closest_idx)

figure(6)
title('Index Anomoly')
hold on
plot(closest_idx(40:60,1))

figure(7)
title('Goal Angle')
hold on
z = zeros(size(x))';
col = goal_angle;  % This is the color, vary with x in this case.
surface([x';x'],[y';y'],[z;z],[col';col'],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',4);
ylabel("Angle (Degrees)");
colorbar

figure(8)
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

figure(9)
title('Angular Velocity')
hold on
z = zeros(size(x))';
col = ang_vel;  % This is the color, vary with x in this case.
surface([x';x'],[y';y'],[z;z],[col';col'],...
        'facecol','no',...
        'edgecol','interp',...
        'linew',4);
ylabel("Angular Velocity");
colorbar

figure(10)
title('Angular Velocity')
hold on
plot(ang_vel)