%% Clear the workspace and the command window.
clear;
clc;

%% Specify the required points of the two-link Robot Arm
A1 = 1;
A2 = 1;
theta1 = 0.2;
theta2 = 0.6;
expX = 0.1;
expY = 1.5;
orgX = 0;
orgY = 0;
errorThreshold = 0.0001;

%Experiment with alpha
alpha = 5;

[expPoint, Joint, nfinal, Theta] = newton_inv_kin(A1, A2, theta1, theta2, expX, expY, orgX, orgY, alpha, errorThreshold);

fprintf('Iteration number: %d\n ', nfinal)

expPoint
[m, n] = size(Joint);
currPoint = [Joint(m,1); Joint(m,2)];
Theta(1,1)
Theta(2,1)
cost = sqrt((expPoint(1,1) - currPoint(1,1))^2 +(expPoint(2,1)-currPoint(2,1))^2);

msgbox('Newton-Raphson Operation Complete')
