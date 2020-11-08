
bags = dir('*.bag'); % this calls/lists all bag files in directory
tester = zeros(length(bags), 2);
topic1 = {'/gazebo/model_states'};

for i = 1:length(bags)
    name = convertStringsToChars(bags(i).name);
    r = strfind(name,'a');
    index = r(length(r)-1);
    split_name = split(name, '_');
    
    run_string = split_name(3);
    run = split(run_string, 'run');
    run = run(2);
    run = str2double(run)
    
    bag = rosbag(bags(i).name);
    bSel1 = select(bag,'Topic', topic1);
    msgStructs1 = readMessages(bSel1,'DataFormat','struct');
    s = size(msgStructs1{1}.Name);
    n = s(1);
    row = [run, n];
    tester(i,:) = row;
end

csvwrite('tester.csv', tester)