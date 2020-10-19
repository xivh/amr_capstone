clear
clc
close all

% Make sure goals.csv and all relevant bag files are in the
% same directory as this script when you run it

bags = dir('*.bag'); % this calls/lists all bag files in directory
training_data = zeros(length(bags), 5);

for i = 1:length(bags)
    %% Collect data from run number
    name = convertStringsToChars(bags(i).name);
    r = strfind(name,'a');
    index = r(length(r)-1);
    n = extractBetween(name,index+1,length(name)-4);
    run = str2num(n{1})
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

    run_list = [1 2 3 4 5 6 7 8 9 10 121 122 123 124 125 126 127 128 129 130 241 242 243 244 245 246 247 248 249 250];
    angle = 0;
    if ismember(run, run_list)
        angle = 0;
        goal_x = 20;
        goal_y = 0;
    elseif ismember(run, run_list+10)
        angle = 15;
        goal_x = 19.659258260000000;
        goal_y = 2.588190451000000;
    elseif ismember(run, run_list+20)
        angle = 30;
        goal_x = 18.660254040000000;
        goal_y = 5;
    elseif ismember(run, run_list+30)
        angle = 45;
        goal_x = 17.071067810000000;
        goal_y = 7.071067812000000;
    elseif ismember(run, run_list+40)
        angle = 60;
        goal_x = 15;
        goal_y = 8.660254038000000;
    elseif ismember(run, run_list+50)
        angle = 75;
        goal_x = 12.588190450000000;
        goal_y = 9.659258263000000;
    elseif ismember(run, run_list+60)
        angle = 90;
        goal_x = 10;
        goal_y = 10;
    elseif ismember(run, run_list+70)
        angle = 105;
        goal_x = 7.411809549000000;
        goal_y = 9.659258263000000;
    elseif ismember(run, run_list+80)
        angle = 120;
        goal_x = 5;
        goal_y = 8.660254038000000;
    elseif ismember(run, run_list+90)
        angle = 135;
        goal_x = 2.928932188000000;
        goal_y = 7.071067812000000;
    elseif ismember(run, run_list+100)
        angle = 150;
        goal_x = 1.339745962000000;
        goal_y = 5;
    elseif ismember(run, run_list+110)
        angle = 165;
        goal_x = 0.340741737100000;
        goal_y = 2.588190451000000;
    end

    % calculate mu
    mu = 0;
    if run <= 120
        mu = 0.009;
    elseif (120 < run) && (run <= 240)
        mu = 0.09;
    elseif 240 < run
        mu = 1;
    end
    
    %% Collect data from bag file
    topic1 = {'/gazebo/model_states'};
    bag = rosbag(bags(i).name);
    bSel1 = select(bag,'Topic', topic1);
    msgStructs1 = readMessages(bSel1,'DataFormat','struct');
    
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
    tolerance = 1;
    success = 0;
    if dist <= tolerance
        success = 1;
    end
    
    row = [run, mu, vel, angle, success];
    training_data(i,:) = row;
end

csvwrite('training_data.csv', training_data)

done = "done"

