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

filtered = [];
curr = [td(1,1), td(1,3), td(1,4), td(1,5)];
max_v = 0.0;
for i = 1:length(td)
    mu = td(i,1);
    v = td(i,2);
    a = td(i,3);
    t = td(i,4);
    s = td(i,5);
    if ([mu, a, t, s] == curr)
        if v > max_v
            max_v = v;
        end
    else
        filtered = [filtered;curr, max_v];
        curr = [mu, a, t, s];
        max_v = v;
    end
end

% need to update this loop here, it removes failures
% should set failure = 0 if no successes for that combo
% also needs to be run until the errors stop...
% for i = 1:length(filtered)
%     if filtered(i,4) == 0
%         filtered(i,:) = []
%     end
% end

% mu, angle, threshold
x_train = [filtered(:,1), filtered(:,2), filtered(:,3)];
% velocity
t_train = filtered(:,5);
writematrix(x_train, 'C:\Users\owner\Desktop\nn test\x_train.csv')
writematrix(t_train, 'C:\Users\owner\Desktop\nn test\t_train.csv')

