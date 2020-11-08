%% Download Data
filenamep = 'C:\Users\owner\Desktop\nn test\predictions.csv';
filenamet = 'C:\Users\owner\Desktop\nn test\training.csv';
preds = csvread(filenamep,1,0);
train = csvread(filenamet,1,0);

%% 3D Plot
mu = preds(:,1);
angle = preds(:,2);
pred_vel = preds(:,4);
figure(1)
plot3(mu, angle, pred_vel)
xlabel("Mu")
ylabel("Angle")
zlabel("Predicted Velocity")
hold off

%% Surface Plot
mu = zeros(60, 1);
angle = 0:1:165;
pred_vel = preds(:,4);
i = 1;
k = 1;
vels = zeros(length(angle), length(mu));
while i <= (length(preds)-165)
    mu(k) = preds(i,1);
    j = i+165;
    vels(:,k) = pred_vel(i:j,1);
    i = i+166;
    k = k+1;
end
figure(2)
surf(mu, angle, vels)
xlabel("Mu")
ylabel("Angle")
zlabel("Predicted Velocity")
hold off

%% All Predicted Data
i = 1;
k = 1;
mus = strings(length(preds)/166, 1);
while i <= (length(preds)-165)
    j = i+165;
    plot(preds(i:j,2), preds(i:j,4))
    hold on
    m = convertCharsToStrings(num2str(preds(i,1)));
    mus(k)= strcat("mu ", m);
    k = k+1;
    i = i+166;
end
l = legend(mus);
l.NumColumns=2;
xlabel("Angle")
ylabel("Predicted Velocity")

%% Plot Predicted vs Trained
i = 1;
m009_pred = [];
m009_pred_1 = []; % prediction with threshold 1
m009_pred_2 = []; % prediction with threshold 2
m09_pred = [];
m5_pred = [];
m1_pred = [];
while i <= (length(preds))
    m = preds(i,1);
    if m == 0.009
        m009_pred = [m009_pred; preds(i,2), preds(i,4)];
        if preds(i,3) == 1.0
            m009_pred_1 = [m009_pred_1; preds(i,2), preds(i,4)]; % fix
        end
        if preds(i,3) == 2.0
            m009_pred_2 = [m009_pred_2; preds(i,2), preds(i,4)]; % fix
        end
    elseif m == 0.09
        m09_pred = [m09_pred; preds(i,2), preds(i,4)];
    elseif m == 0.5
        m5_pred = [m5_pred; preds(i,2), preds(i,4)];
    elseif m == 1
        m1_pred = [m1_pred; preds(i,2), preds(i,4)];
    end
    i = i + 1;
end

k = 1;
m009_train = [];
m009_train_1 = []; % training with threshold 1
m009_train_2 = []; % training with threshold 1
m09_train = [];
m5_train = [];
m1_train = [];
while k <= (length(train))
    m = train(k,1);
    if m == 0.009
        m009_train = [m009_train; train(k,2) train(k,4)];
        if train(k,3) == 1.0
            m009_train_1 = [m009_train_1; train(k,2), train(k,4)]; % fix
        end
        if train(k,3) == 2.0
            m009_train_2 = [m009_train_2; train(k,2), train(k,4)]; % fix
        end
    elseif m == 0.09
        m09_train = [m09_train; train(k,2) train(k,4)];
    elseif m == 0.5
        m5_train = [m5_train; train(k,2) train(k,4)];
    elseif m == 1
        m1_train = [m1_train; train(k,2) train(k,4)];
    end
    k = k+1;
end

% Mu = 0.009
% average over threshold
[ud, ix, iy] = unique(m009_pred(:,1));
m009_pred_avg = [ud, accumarray(iy, m009_pred(:,2),[],@mean)];
[ud, ix, iy] = unique(m009_train(:,1));
m009_train_avg = [ud, accumarray(iy, m009_train(:,2),[],@mean)];
figure(3)
plot(m009_pred_avg(:,1), m009_pred_avg(:,2))
hold on
plot(m009_train_avg(:,1), m009_train_avg(:,2))
title("Mu = 0.009")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off

% Mu = 0.09
% average over threshold
[ud, ix, iy] = unique(m09_pred(:,1));
m09_pred_avg = [ud, accumarray(iy, m09_pred(:,2),[],@mean)];
[ud, ix, iy] = unique(m09_train(:,1));
m09_train_avg = [ud, accumarray(iy, m09_train(:,2),[],@mean)];
figure(4)
plot(m09_pred_avg(:,1), m09_pred_avg(:,2))
hold on
plot(m09_train_avg(:,1), m09_train_avg(:,2))
title("Mu = 0.09")
xlabel("Angle")
ylabel("Predicted Velocity")
ylim([1 3])
legend("Predicted", "Training Data")
hold off

% Mu = 0.5
% average over threshold
[ud, ix, iy] = unique(m5_pred(:,1));
m5_pred_avg = [ud, accumarray(iy, m5_pred(:,2),[],@mean)];
[ud, ix, iy] = unique(m5_train(:,1));
m5_train_avg = [ud, accumarray(iy, m5_train(:,2),[],@mean)];
figure(5)
plot(m5_pred_avg(:,1), m5_pred_avg(:,2))
hold on
plot(m5_train_avg(:,1), m5_train_avg(:,2))
title("Mu = 0.5")
xlabel("Angle")
ylabel("Predicted Velocity")
ylim([1 3])
legend("Predicted", "Training Data")
hold off

% Mu = 1
% average over threshold
[ud, ix, iy] = unique(m1_pred(:,1));
m1_pred_avg = [ud, accumarray(iy, m1_pred(:,2),[],@mean)];
[ud, ix, iy] = unique(m1_train(:,1));
m1_train_avg = [ud, accumarray(iy, m1_train(:,2),[],@mean)];
figure(6)
plot(m1_pred_avg(:,1), m1_pred_avg(:,2))
hold on
plot(m1_train_avg(:,1), m1_train_avg(:,2))
title("Mu = 1")
xlabel("Angle")
ylabel("Predicted Velocity")
ylim([1 3])
legend("Predicted", "Training Data")
hold off

%% Plot with threshold
% Mu = 0.009
% sort - do this earlier for everything...
m009_train_1 = sortrows(m009_train_1);
% average
[ud, ix, iy] = unique(m009_train_1(:,1));
m009_train_1_avg = [ud, accumarray(iy, m009_train_1(:,2),[],@mean)];
figure(7)
plot(m009_pred_1(:,1), m009_pred_1(:,2))
hold on
plot(m009_train_1_avg(:,1), m009_train_1_avg(:,2))
title("Mu = 0.009, threshold = 1 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off

% Mu = 0.009, 2 thresholds
% sort - do this earlier for everything...
m009_train_2 = sortrows(m009_train_2);
% average
[ud, ix, iy] = unique(m009_train_2(:,1));
m009_train_2_avg = [ud, accumarray(iy, m009_train_2(:,2),[],@mean)];

figure(8)
plot(m009_pred_1(:,1), m009_pred_1(:,2))
hold on
plot(m009_train_1_avg(:,1), m009_train_1_avg(:,2))
plot(m009_pred_2(:,1), m009_pred_2(:,2))
plot(m009_train_2_avg(:,1), m009_train_2_avg(:,2))
title("Mu = 0.009, thresholds = 1 and 2 m")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted (1 m)", "Training (1 m)", "Predicted (2 m)", "Training (2 m)")
%legend("Predicted", "Training Data")
hold off

