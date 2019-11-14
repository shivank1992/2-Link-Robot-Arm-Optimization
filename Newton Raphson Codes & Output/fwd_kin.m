function [Jacobian, Joint] = fwd_kin(A1, A2, theta1, theta2, draw, orgX, orgY)
% Forward Kinematics for 2 link Robotic Arm movement optimization:
% Written by Shivank Garg
% Date: Nov 5, 2019


%% Modelling of the arm
%% A1 and A2 are the lengths of the respective links
X(1)= orgX+ A1*cosd(theta1);
Y(1)= orgY+ A1*sind(theta1);

%% End Effector Coordinates : Page-1
X(2)= X(1)+ A2*cosd(theta1 + theta2); % Page-1 Eq1
Y(2)= Y(1)+ A2*sind(theta1 + theta2);  %Page-1 Eq2

% Storing the Joint Locations of the arm
Joint = [orgX, orgY; X(1), Y(1); X(2), Y(2)]; % X(1), Y(1) is the location of arm 1 end point and X(1), Y(1) is the end location of arm 2

% Jacobian of the forward kinematics function : Page-2
% Storing the Jacobian matrix for the current arm configuration
Jacobian = [-A1*sind(theta1) - A2*sind(theta1+theta2), -A2*sind(theta1+theta2);
    A1*cosd(theta1)+ A2*cosd(theta1+theta2), A2*cosd(theta1+theta2)];

% If it was mentioned in the parameter 'draw' then draw the arm.
if draw == true
    plot([orgX,X(1)],[orgY,Y(1)]);
    hold on
    plot([X(1),X(2)],[Y(1),Y(2)]);
    title('Two link Robot Arm Optimization: Newton Raphson');
    legend('1st Link of Arm', '2nd Link of Arm');
end

end

