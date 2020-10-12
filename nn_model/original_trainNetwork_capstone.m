clc
clear
close all
 
load('distance.mat')
load('safety_label.mat')
load('position.mat')

position_2d = initial_pos_2d';
distance = avoid_dist;
safety_label = single_safety_label_save;

%%
% definition of the training inputs and outputs.
% as this is a binary classification, the t matrix is of size 2xm where m
% is the trainin data size. If the training data belongs to class 1, first
% row of the t matrix would be 1, otherwise second 
x  = [position_2d ; distance];
t  = [safety_label; ~safety_label];

% definition of performance function
performFcn = 'crossentropy';
% hidden layer size 
% in this case, there are two layers with 50 nodes each
hiddenSizes = [50, 50];

% definition of train function and the activation function
trainFcn = 'trainrp';
net = patternnet(hiddenSizes,trainFcn, performFcn);
net.layers{1}.transferFcn = 'logsig';
net.layers{2}.transferFcn = 'logsig';

% trainin of the network for given input-output pairs
net = train(net,x,t);
% visualize the structure of your network
view(net);

%performance of the trained network on the training inputs
% you can change x to any input that you want to test
y = net(x) - t;
y = y(1,:);
std(y)




