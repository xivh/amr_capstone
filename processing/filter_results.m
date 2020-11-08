% you have to use this like a live script and run sections because there
% are errors in the commented out loops

% td = training_data
% or
% td = csvread('training_data.csv')

% td = td(:,2:end)

% run this a couple times until the index error is gone
% for i = 1:length(td)
% if td(i,1) == -1;
% td(i,:) = []
% end
% end

% current logic
% 1. filter everything to get max velocity for each combination of mu, angle,
% threshold, success - so max success velocity, max fail velocity
% 2. if there is a failure in the filtered dataset, set the velocity to 0
% for that combo of mu, angle, threshold
%
% problem with this logic: sometimes it succeeds at a higher velocity than
% it fails (example: 0.5/75/1.5 fails at 0.2 m/s, succeeds above until max
% recorded run, 1.8 m/s

td = sortrows(td, [1 3 4 5])
filtered = []; % keep max velocity - (mu, a, t, success, velocity)
successes = []; % this could be a hash table or something
failures = []; % this could be a hash table or something
curr = [td(1,1), td(1,3), td(1,4), td(1,5)]; % current run combo (mu, a, t, success)
prev = curr; % previous run combo (mu, a, t, success)
max_v = 0.0;
for i = 1:length(td) % this only works on sorted arrays - also should start @ 2?
    mu = td(i,1);
    v = td(i,2);
    a = td(i,3);
    t = td(i,4);
    s = td(i,5);
    if ([mu, a, t, s] == curr)
        'a'
        if v > max_v
            max_v = v;
        end
    else
        filtered = [filtered;prev, max_v]; % could fail if prev is empty
        if s == 0
            failures = [failures; prev(1:3)]; % store failure (mu, a, t)
        elseif s == 1
            successes = [successes; prev(1:3)];
        end
        curr = [mu, a, t, s];
        prev = curr;
        max_v = v;
    end
end

condensed = [-1, -1, -1, -1]; % remove duplicates
for i = 1:length(filtered)
    is_success = sum(ismember(successes, filtered(i, 1:3), 'rows'));
    is_failure = sum(ismember(failures, filtered(i, 1:3), 'rows'));
    if sum(ismember(condensed(:,1:3), filtered(i, 1:3), 'rows'))
        continue % do not add to condensed if already there
    elseif is_success & is_failure % could be replaced w/ is_failure only?
        condensed = [condensed; filtered(i, 1:3), 0];
    elseif is_success
        condensed = [condensed; filtered(i, 1:3), filtered(i, 5)];
    elseif is_failure
        condensed = [condensed; filtered(i, 1:3), 0];
    end
end
condensed = condensed(2:end,:); % because my first if statement fails on empty array

% mu, angle, threshold
x_train = [condensed(:,1), condensed(:,2), condensed(:,3)];
% velocity
t_train = condensed(:,4);
writematrix(x_train, 'C:\Users\owner\Desktop\nn test\x_train.csv')
writematrix(t_train, 'C:\Users\owner\Desktop\nn test\t_train.csv')

