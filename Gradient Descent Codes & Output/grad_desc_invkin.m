function [expPoint, Joint, nfinal] = grad_desc_invkin(A1, A2, theta1, theta2, expX, expY, orgX, orgY, alpha, errorThreshold)
% Gradient Descent method for 2 link Robotic Arm movement optimization:
% Written by Shivank Garg
% Date: Nov 6, 2019


% Variables to store iterations related data
nfinal = 0;

% Matrix containing Joint angle Information and Desired Point Location
Theta = [theta1; theta2];
expPoint = [expX; expY];%Desired location of the arm end point

%% Inverse Kinematics Initialization

% Draw the original position of the 2-link arm
[Jacobian, Joint] = fwd_kin(A1, A2, theta1, theta2, true, orgX, orgY);
[m, n] = size(Joint);
currPoint = [Joint(m,1); Joint(m,2)];

% Get the error vector representing difference between current and desired
% position of the arm
error = (currPoint - expPoint);

% Apply the control equation to the error vector
controlError = error * alpha;
deltaTheta = transpose(Jacobian) * controlError;

% Apply the control changes to the Joint Angles
Theta(1,1) = Theta(1,1) - deltaTheta(1,1);
Theta(2,1) = Theta(2,1) - deltaTheta(2,1);

% Find Cost : the distance between the desired and current position of the arm.
cost = sqrt((expPoint(1,1) - currPoint(1,1))^2 +(expPoint(2,1)-currPoint(2,1))^2)

%% Iterative Inverse Kinematics for correcting the arm position based on Gradient Descent

% Correct the arm position to reduce the distance.
while (abs(cost) > errorThreshold)
	nfinal = nfinal + 1; % Count iteration
    [Jacobian, Joint] = fwd_kin(A1, A2, Theta(1,1), Theta(2,1), true, orgX, orgY); % Calculate Jacobian
    currPoint = [Joint(m,1); Joint(m,2)]; % Calculate the f(qk)
    error = (currPoint - expPoint); % Calculate the error between expected and current position
    controlError = error * alpha; % alphak *(f(qk) - xd)
    deltaTheta = transpose(Jacobian) * controlError; %transpose(J)*alphak *(f(qk) - xd)
    Theta(1,1) = Theta(1,1) - deltaTheta(1,1); % Apply the changes to theta 1
    Theta(2,1) = Theta(2,1) - deltaTheta(2,1); % Apply the changes to theta 2
    cost = sqrt((expPoint(1,1) - currPoint(1,1))^2 +(expPoint(2,1)-currPoint(2,1))^2) % Calculate cost
end

end