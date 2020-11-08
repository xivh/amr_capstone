clear
clc
close all

% Make sure goals.csv and all relevant bag files are in the
% same directory as this script when you run it

bags = dir('*.bag'); % this calls/lists all bag files in directory
training_data = zeros(length(bags), 6);

for i = 1:length(bags)
    %% Collect data from run number
    name = convertStringsToChars(bags(i).name);
    r = strfind(name,'a');
    index = r(length(r)-1);
    %n = extractBetween(name,index+1,length(name)-4);
    %run = str2num(n{1})
    name_starting_from_run = name(strfind(name,'run'):end);
    split_name = split(name_starting_from_run, '_');
    run_string = split_name(1);
    run = split(run_string, 'run');
    run = run(2);
    run = str2double(run)
    
    name_starting_from_angle = name(strfind(name,'angle'):end);
    split_name = split(name_starting_from_angle, '.');
    angle_string = split_name(1);
    a = split(angle_string, 'angle');
    a = a(2);
    angle = str2double(a)
    
    i
    % calculate vel
    v_num = mod(run, 10);
    if v_num == 1
        vel = 0.2;
    elseif v_num == 2
        vel = 0.4;
    elseif v_num == 3
        vel = 0.6;
    elseif v_num == 4
        vel = 0.8;
    elseif v_num == 5
        vel = 1.0;
    elseif v_num == 6
        vel = 1.2;
    elseif v_num == 7
        vel = 1.4;
    elseif v_num == 8
        vel = 1.6;
    elseif v_num == 9
        vel = 1.8;
    elseif v_num == 0
        vel = 2.0;
    end

    %run_list = [1 2 3 4 5 6 7 8 9 10 121 122 123 124 125 126 127 128 129 130 241 242 243 244 245 246 247 248 249 250];
    %angle = 0;
%     if ismember(run, run_list)
%         angle = 0;
%         goal_x = 20;
%         goal_y = 0;
%     elseif ismember(run, run_list+10)
%         angle = 15;
%         goal_x = 19.659258260000000;
%         goal_y = 2.588190451000000;
%     elseif ismember(run, run_list+20)
%         angle = 30;
%         goal_x = 18.660254040000000;
%         goal_y = 5;
%     elseif ismember(run, run_list+30)
%         angle = 45;
%         goal_x = 17.071067810000000;
%         goal_y = 7.071067812000000;
%     elseif ismember(run, run_list+40)
%         angle = 60;
%         goal_x = 15;
%         goal_y = 8.660254038000000;
%     elseif ismember(run, run_list+50)
%         angle = 75;
%         goal_x = 12.588190450000000;
%         goal_y = 9.659258263000000;
%     elseif ismember(run, run_list+60)
%         angle = 90;
%         goal_x = 10;
%         goal_y = 10;
%     elseif ismember(run, run_list+70)
%         angle = 105;
%         goal_x = 7.411809549000000;
%         goal_y = 9.659258263000000;
%     elseif ismember(run, run_list+80)
%         angle = 120;
%         goal_x = 5;
%         goal_y = 8.660254038000000;
%     elseif ismember(run, run_list+90)
%         angle = 135;
%         goal_x = 2.928932188000000;
%         goal_y = 7.071067812000000;
%     elseif ismember(run, run_list+100)
%         angle = 150;
%         goal_x = 1.339745962000000;
%         goal_y = 5;
%     elseif ismember(run, run_list+110)
%         angle = 165;
%         goal_x = 0.340741737100000;
%         goal_y = 2.588190451000000;
%     end

% calculate goals from angle
    if angle == 0;
        goal_x = 20;
        goal_y = 0;
    elseif angle == 15;
        goal_x = 19.659258260000000;
        goal_y = 2.588190451000000;
    elseif angle == 30;
        goal_x = 18.660254040000000;
        goal_y = 5;
    elseif angle == 45;
        goal_x = 17.071067810000000;
        goal_y = 7.071067812000000;
    elseif angle == 60;
        goal_x = 15;
        goal_y = 8.660254038000000;
    elseif angle == 75;
        goal_x = 12.588190450000000;
        goal_y = 9.659258263000000;
    elseif angle == 90;
        goal_x = 10;
        goal_y = 10;
    elseif angle == 105;
        goal_x = 7.411809549000000;
        goal_y = 9.659258263000000;
    elseif angle == 120;
        goal_x = 5;
        goal_y = 8.660254038000000;
    elseif angle == 135;
        goal_x = 2.928932188000000;
        goal_y = 7.071067812000000;
    elseif angle == 150;
        goal_x = 1.339745962000000;
        goal_y = 5;
    elseif angle == 165;
        goal_x = 0.340741737100000;
        goal_y = 2.588190451000000;
    end
    
    % calculate mu
    mu = 0;
    if run <= 960
        mu = 0.009;
    elseif (960 < run) && (run <= 1920)
        mu = 0.09;
    elseif (1920 < run) && (run <= 2880)
        mu = 1;
    elseif (2880 < run) && (run <= 3840)
        mu = 0.05;
    elseif 3840 < run
        mu = 0.5;
    end

    % calculate tolerance threshold
    t_num = mod(run, 960);
    tolerance = 0;
    if t_num <= 120
        tolerance = 0.5;
    elseif (120 < t_num) && (t_num <= 240)
        tolerance = 1;
    elseif (240 < t_num) && (t_num <= 360)
        tolerance = 1.5;
    elseif (360 < t_num) && (t_num <= 480)
        tolerance = 2;
    elseif (480 < t_num) && (t_num <= 600)
        tolerance = 2.5;
    elseif (600 < t_num) && (t_num <= 720)
        tolerance = 3;
    elseif (720 < t_num) && (t_num <= 840)
        tolerance = 3.5;
    elseif (840 < t_num) && (t_num <= 960)
        tolerance = 4;
    end

    %% Collect data from bag file
    topic1 = {'/gazebo/model_states'};
    bag = rosbag(bags(i).name);
    bSel1 = select(bag,'Topic', topic1);
    msgStructs1 = readMessages(bSel1,'DataFormat','struct');
    
    try
        % Get index of robot position
        n1 = cellfun(@(m) (m.Name(1)),msgStructs1)';
        n2 = cellfun(@(m) (m.Name(2)),msgStructs1)';
        names = [n1(1), n2(1)];
        isRobot = cellfun(@(x)isequal(x,'jackal'),names);
        [row,index] = find(isRobot);

        % Extract Data
        x = cellfun(@(m) (m.Pose(index).Position.X),msgStructs1)';
        y = cellfun(@(m) (m.Pose(index).Position.Y),msgStructs1)';
        x_final = x(1, length(x));
        y_final = y(1, length(y));
        % filename = 'C:\Users\Student\Documents\UVA\Year 4\Capstone\bag_reader_capstone\Training Bags 10.18\goals.csv';
        % goals = importdata(filename);
        % s = size(goals);
        % rows = size(1);
        % for j = 1:rows
        %    if angle == goals(j,1)
        %        goal_x = goals(j,2);
        %        goal_y = goals(j,3);
        %    end
        % end
        dist = sqrt((goal_x - x_final).^2 + (goal_y - y_final).^2);

        % calculate success
        %tolerance = 1;
        success = 0;
        if dist <= tolerance
            success = 1;
        end

        row = [run, mu, vel, angle, tolerance, success];
        training_data(i,:) = row;
    catch
        fprintf('run %i failed\n', run)
        row = [run, -1, -1, -1, -1, -1];
        training_data(i,:) = row;
    end
end

csvwrite('training_data.csv', training_data)
done = "done"

