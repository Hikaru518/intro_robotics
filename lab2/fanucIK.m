function [is_solution,joint_angles] = fanucIK(T,prev_joint_angles,fanuc)
% Takes as its inputs a transform T describing the desired position and 
% orientation of the end effector (NOT the tool), a 6-element vector of 
% the previous FANUC joint angles, and the structure output by the 
% function fanucInit, and returns the Boolean variable is_solution,
% and the 6-element vector joint_angles

% parameter setting
alpha = [0,pi/2,0,pi/2,-pi/2,pi/2];
a = [0,300,900,180,0,0];
d = [0,0,0,1600,0,180];
l6 = 180;

is_solution = true;

% workspace checking
% endeff_x = T(1,4);
% endeff_y = T(2,4);
% endeff_z = T(3,4);
% if sqrt(endeff_x^2+endeff_y^2)>2739 || endeff_x<-2739*cos(pi/6) || endeff_z>3238 || endeff_z<-721 
%     is_solution = false;
%     return;
% end

% solve for theta1, theta2, theta3
p5 = inv(T)*([0,0,-l6,1]');
Point5 = [p5(1),p5(2),p5(3)];
x5 = Point5(1);
y5 = Point5(2);
z5 = Point5(3);

% theta1 = atan(y5/x5);
%%%%%%%%%%%%%%%%%%%%%% I have some doubt about the following code
theta1=atan(y5/x5);
% if round(sin(theta1)*1000)/1000 == 0
%     if round(theta1*1000)/1000 ~= 0
%         if theta1 > 0
%             theta1 = theta1 - pi;
%         end
%         if theta1 < 0
%             theta1 = theta1 + pi;
%         end
%     end
% end
%%%%%%%%%%%%%%%%%%%%%%%

l1_tmp = 900;
l2_tmp = sqrt(180*180+1600*1600);
alpha_tmp = atan2(1600,180);

tmp_a = sqrt(x5.^2+y5.^2)-300;
tmp_b = z5;
c3 = (tmp_a.^2 + tmp_b.^2-l1_tmp.^2-l2_tmp.^2)/(2*l1_tmp*l2_tmp);
theta3_1 = acos(c3); % one of the solution
theta3_2 = -theta3_1;

beta = atan(tmp_b/tmp_a);
Cphi = (tmp_a.^2 + tmp_b.^2+l1_tmp.^2-l2_tmp.^2)/(2*l1_tmp*sqrt(tmp_a.^2 + tmp_b.^2));
phi = acos(Cphi);

if(theta3_1>0)
    theta2_1 = beta - phi;
    theta2_2 = beta + phi;
else
    theta2_1 = beta + phi;
    theta2_2 = beta + phi;
end

theta3_a = theta3_1 + alpha_tmp;
theta3_b = theta3_2 + alpha_tmp;
theta2_a = theta2_1 - pi/2;
theta2_b = theta2_2 - pi/2;

list_tmp = [theta3_a,theta3_b,theta2_a,theta3_b];
for ii = 1:4
    if(list_tmp(ii)>2*pi)
        list_tmp(ii) = list_tmp(ii) - 2*pi;
    elseif(list_tmp(ii)<-2*pi)
        list_tmp(ii) = list_tmp(ii) + 2*pi;
    end
end

theta3_a = list_tmp(1);
theta3_b = list_tmp(2);
theta2_a = list_tmp(3);
theta2_b = list_tmp(4);

% solve for theta4, theta5, theta6 :chose theta2_a and theta3_a
theta2 = theta2_a;
theta3 = theta3_a;
T01 = dhtf(alpha(1),a(1),d(1),theta1);
T12 = dhtf(alpha(2),a(2),d(2),theta2);
T23 = dhtf(alpha(3),a(3),d(3),theta3);
T03 = T01*T12*T23;
T36 = T03*T;

Point3 = T36(1:3,4);
x3 = Point3(1); y3 = Point3(2); z3 = Point3(3);
theta4 = atan(y3/(x3-180));

T34 = dhtf(alpha(4),a(4),d(4),theta4);
T04 = T03*T34;
T46 = T04*T;

Point4 = T46(1:3,4);
x4 = Point4(1); y4 = Point4(2); z4 = Point4(3);
theta5 = atan(x4/z4);

T45 = dhtf(alpha(5),a(5),d(5),theta5);
T05 = T04*T45;
R05 = T05(1:3,1:3);
R06 = T(1:3,1:3);
Ttmp = dhtf(alpha(6),a(6),d(6),0);
Rtmp = Ttmp(1:3,1:3);
Rz = inv(Rtmp)*inv(R05)*R06;
theta6 = acos(Rz(1,1));

theta_solution1 = [theta1, theta2, theta3, theta4, theta5, theta6];

% solve for theta4, theta5, theta6 :chose theta2_b and theta3_b
theta2 = theta2_b;
theta3 = theta3_b;
T01 = dhtf(alpha(1),a(1),d(1),theta1);
T12 = dhtf(alpha(2),a(2),d(2),theta2);
T23 = dhtf(alpha(3),a(3),d(3),theta3);
T03 = T01*T12*T23;
T36 = T03*T;

Point3 = T36(1:3,4);
x3 = Point3(1); y3 = Point3(2); z3 = Point3(3);
theta4 = atan(y3/(x3-180));

T34 = dhtf(alpha(4),a(4),d(4),theta4);
T04 = T03*T34;
T46 = T04*T;

Point4 = T46(1:3,4);
x4 = Point4(1); y4 = Point4(2); z4 = Point4(3);
theta5 = atan(x4/z4);

T45 = dhtf(alpha(5),a(5),d(5),theta5);
T05 = T04*T45;
R05 = T05(1:3,1:3);
R06 = T(1:3,1:3);
Ttmp = dhtf(alpha(6),a(6),d(6),0);
Rtmp = Ttmp(1:3,1:3);
Rz = R06*inv(R05)*inv(Rtmp);
theta6 = acos(Rz(1,1));

theta_solution2 = [theta1, theta2, theta3, theta4, theta5, theta6];

% chose a nearest one
distance1 = norm(prev_joint_angles-theta_solution1);
distance2 = norm(prev_joint_angles-theta_solution2);
if(distance1 < distance2)
    joint_angles = theta_solution1;
else
    joint_angles = theta_solution2;
end

% check joint limit
%%%%%%TO DO%%%%%%
% for ii = 1:6
%     limit = fanuc.joint_limits{ii};
%     if(joint_angles(ii)<limit(1) || joint_angles(ii)>limit(2))
%         is_solution = false;
%         return;
%     end
% end
%%%%%%TO DO%%%%%%

end

