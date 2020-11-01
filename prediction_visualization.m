%% Download Data
filenamep = 'C:\Users\Student\Documents\UVA\Year 4\Capstone\predictions.csv';
filenamet = 'C:\Users\Student\Documents\UVA\Year 4\Capstone\training.csv';
preds = csvread(filenamep);
train = csvread(filenamet);

%% 3D Plot
mu = preds(:,1);
angle = preds(:,2);
pred_vel = preds(:,3);
figure(1)
plot3(mu, angle, pred_vel)
xlabel("Mu")
ylabel("Angle")
zlabel("Predicted Velocity")
hold off

%% Surface Plot
mu = zeros(60, 1);
angle = 0:1:165;
pred_vel = preds(:,3);
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
    plot(preds(i:j,2), preds(i:j,3))
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
while i <= (length(preds)-165)
    j = i+165;
    m = preds(i,1);
    if m == 0.009
        m009_pred = [preds(i:j,2) preds(i:j,3)];
    elseif m == 0.09
        m09_pred = [preds(i:j,2) preds(i:j,3)];
    elseif m == 0.5
        m5_pred = [preds(i:j,2) preds(i:j,3)];
    elseif m == 1
        m1_pred = [preds(i:j,2) preds(i:j,3)];
    end
    i = i+166;
end

k = 1;
while k <= (length(train)-11)
    j = k+11;
    m = train(k,1);
    if m == 0.009
        m009_train = [train(k:j,2) train(k:j,3)];
    elseif m == 0.09
        m09_train = [train(k:j,2) train(k:j,3)];
    elseif m == 0.5
        m5_train = [train(k:j,2) train(k:j,3)];
    elseif m == 1
        m1_train = [train(k:j,2) train(k:j,3)];
    end
    k = k+12;
end
% Mu = 0.009
figure(3)
plot(m009_pred(:,1), m009_pred(:,2))
hold on
plot(m009_train(:,1), m009_train(:,2))
title("Mu = 0.009")
xlabel("Angle")
ylabel("Velocity")
ylim([0 2])
legend("Predicted", "Training Data")
hold off

% Mu = 0.09
figure(4)
plot(m09_pred(:,1), m09_pred(:,2))
hold on
plot(m09_train(:,1), m09_train(:,2))
title("Mu = 0.09")
xlabel("Angle")
ylabel("Predicted Velocity")
ylim([1 3])
legend("Predicted", "Training Data")
hold off

% Mu = 0.5
figure(5)
plot(m5_pred(:,1), m5_pred(:,2))
hold on
plot(m5_train(:,1), m5_train(:,2))
title("Mu = 0.5")
xlabel("Angle")
ylabel("Predicted Velocity")
ylim([1 3])
legend("Predicted", "Training Data")
hold off

% Mu = 1
figure(6)
plot(m1_pred(:,1), m1_pred(:,2))
hold on
plot(m1_train(:,1), m1_train(:,2))
title("Mu = 1")
xlabel("Angle")
ylabel("Predicted Velocity")
ylim([1 3])
legend("Predicted", "Training Data")
hold off