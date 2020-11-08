clear
clc
close all

tic

topic1 = {'/gazebo/model_states'};
topic2 = {'/odometry/filtered'};
  
%bag1 = rosbag('ice_test.bag');
%bag2 = rosbag('control_test.bag');
path = '' % set the path to the directory with bag files, end with /
bags = dir(strcat(path,'*.bag')); % this calls/lists all bag files in directory

for i = 1:length(bags)

    bag = rosbag(strcat(path,bags(i).name));
    bSel1 = select(bag,'Topic', topic1);
    %bSel2 = select(bag, 'Topic', topic2);
    msgStructs1 = readMessages(bSel1,'DataFormat','struct');
    %msgStructs2 = readMessages(bSel2,'DataFormat','struct');

    % Get index of robot position
    n1 = cellfun(@(m) (m.Name(1)),msgStructs1)';
    n2 = cellfun(@(m) (m.Name(2)),msgStructs1)';
    % n3 = cellfun(@(m) (m.Name(3)),msgStructs1)';
    % n4 = cellfun(@(m) (m.Name(4)),msgStructs1)';
    % n5 = cellfun(@(m) (m.Name(5)),msgStructs1)';
    % n6 = cellfun(@(m) (m.Name(6)),msgStructs1)';
    % n7 = cellfun(@(m) (m.Name(7)),msgStructs1)';
    % n8 = cellfun(@(m) (m.Name(8)),msgStructs1)';
    names = [n1(1), n2(1)];%, n3(1), n4(1), n5(1), n6(1), n7(1), n8(1)];
    isRobot = cellfun(@(x)isequal(x,'jackal'),names);
    [row,index] = find(isRobot);

    % Extract Data
    x = cellfun(@(m) (m.Pose(index).Position.X),msgStructs1)';
    y = cellfun(@(m) (m.Pose(index).Position.Y),msgStructs1)';
    lin_v = cellfun(@(m) (m.Twist(index).Linear.X),msgStructs1)';
    ang_v = cellfun(@(m) (m.Twist(index).Angular.Z),msgStructs1)';
    %x_odom = cellfun(@(m) (m.Pose.Pose.Position.X),msgStructs2)';
    %y_odom = cellfun(@(m) (m.Pose.Pose.Position.Y),msgStructs2)';
    %lin_v_odom = cellfun(@(m) (m.Twist.Twist.Linear.X),msgStructs2)';
    %ang_v_odom = cellfun(@(m) (m.Twist.Twist.Angular.Z),msgStructs2)';
    
    % Plot Data
    figure(1)
    title('Robot Paths')
    hold on
    plot(x,y,'-','LineWidth',2)
    hold on
    grid on

    % figure(2)
    % title('Robot Velocity')
    % hold on
    % plot(lin_v,'-','LineWidth',2)
    % hold on
    % plot(ang_v,'-','LineWidth',2)
    % legend('Linear Velocity','Angular Velocity')
    % grid on

end
