% Name:
%   Peiguang Wang
%   Sichao Zhang

function [ handles ] = drawFanuc( joint_angles, fanuc )
% MECH 498/598 - Intro to Robotics - Spring 2016
% Lab 2 - Inverse Kinematics
%
%
%    DESCRIPTION - Plot a graphical representation of the FANUC S-500
%    Industrial robot with attached coordinate frames, including the tool
%    attached to the end effector.
%
%    INPUTS - joint_angles is a 6-element vector of FANUC joint angles to
%                  specify the pose in which we wish to draw the FANUC.
%    
%             fanuc is a structure generated by fanucInit()
%
%    OUTPUTS - handles is a vector of graphics handles corresponding to the
%                  moving frames attached to the robot
%
%    ADDITIONAL CODE NEEDED:
%   
%    The function fanucFK() must be completed in order to run this function.
%
%    Make sure you understand the section of code marked "Draw Robot", and
%    duplicate for the remaining links what has already been done for links
%    0 and 1. The tool has also been handled for you.
%
%%
% Create a cell array of FANUC forward kinematics transforms
[~,fanuc_T] = fanucFK(joint_angles,fanuc);

% Shorten variable names
l_1 = fanuc.parameters.l_1;
l_2 = fanuc.parameters.l_2;
l_3 = fanuc.parameters.l_3;
l_4 = fanuc.parameters.l_4;
l_5 = fanuc.parameters.l_5;
l_6 = fanuc.parameters.l_6;
l_t = fanuc.parameters.l_t;
l_t_rad = fanuc.parameters.l_t_rad;

% Plot scaling properties
origin_size = 20;
marker_size = 10;
vector_size = 0.05*max(abs(diff(reshape(fanuc.workspace,2,3))));

% Create figure window
figure('Color','w');

% Create axes object
ax = axes('XLim',fanuc.workspace(1:2),'YLim',fanuc.workspace(3:4),...
   'ZLim',fanuc.workspace(5:6));
vw = [31.3,22.8];
set(gca,'View',vw);
grid on;
axis equal;
xlabel('X (mm)','FontSize',16);
ylabel('Y (mm)','FontSize',16);
zlabel('Z (mm)','FontSize',16);
%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Draw Robot %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Create link 0 and frame 0
h = drawRobotFrame([0,0,0]);
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
circ = linspace(0,2*pi,50);
L_0 = line(100*cos(circ),100*sin(circ),...
    -l_1*ones(length(circ)),...
    'Color','k','LineWidth',1.5);
set(L_0,'Parent',hg);
T_0 = hgtransform('Parent',ax,'Matrix',makehgtform('translate',[0,0,l_1]));
set(hg,'Parent',T_0);

% Create link 1 and frame 1
h = drawRobotFrame(fanuc.colors{1});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_1 = line([0,0,l_2],[0,0,0],[-l_1,0,0],...
    'Color',fanuc.colors{1},'LineWidth',1.5);
set(L_1,'Parent',hg);
T_1 = hgtransform('Parent',T_0,'Matrix',fanuc_T{1});
set(hg,'Parent',T_1);

% Create link 2 and frame 2
h = drawRobotFrame(fanuc.colors{2});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_2 = line([0,l_3],[0,0],[0,0],...
    'Color',fanuc.colors{2},'LineWidth',1.5);
set(L_2,'Parent',hg);
T_2 = hgtransform('Parent',T_1,'Matrix',fanuc_T{2});
set(hg,'Parent',T_2);

% Create link 3 and frame 3
h = drawRobotFrame(fanuc.colors{3});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_3 = line([0,l_4,l_4],[0,0,-l_5],[0,0,0],...
    'Color',fanuc.colors{3},'LineWidth',1.5);
set(L_3,'Parent',hg);
T_3 = hgtransform('Parent',T_2,'Matrix',fanuc_T{3});
set(hg,'Parent',T_3);

% Create link 4 and frame 4
h = drawRobotFrame(fanuc.colors{4});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_4 = line([0,0,0],[0,0,0],[0,0,0],...
    'Color',fanuc.colors{4},'LineWidth',1.5);
set(L_4,'Parent',hg);
T_4 = hgtransform('Parent',T_3,'Matrix',fanuc_T{4});
set(hg,'Parent',T_4);

% Create link 5 and frame 5
h = drawRobotFrame(fanuc.colors{5});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_5 = line([0,0],[0,-l_6],[0,0],...
    'Color',fanuc.colors{5},'LineWidth',1.5);
set(L_5,'Parent',hg);
T_5 = hgtransform('Parent',T_4,'Matrix',fanuc_T{5});
set(hg,'Parent',T_5);

% Create link 6 and frame 6
h = drawRobotFrame(fanuc.colors{6});
hg = hggroup('Parent',ax);
set(h,'Parent',hg);
L_6 = line([0,0],[0,-l_6],[0,0],...
    'Color',fanuc.colors{5},'LineWidth',1.5);
set(L_6,'Parent',hg);
T_6 = hgtransform('Parent',T_5,'Matrix',fanuc_T{6});
set(hg,'Parent',T_6);

% Tool base frame (coincident with frame 6)
hg = hggroup('Parent',ax);
L_t = line(50*cos(circ),50*sin(circ),...
    zeros(length(circ)),...
    'Color',fanuc.colors{7},'LineWidth',1.5);
set(L_t,'Parent',hg);
L_t1 = line([l_t_rad,l_t_rad+l_t/sqrt(2)]*cos(pi/4),[l_t_rad,l_t_rad+l_t/sqrt(2)]*sin(pi/4),[0,l_t/sqrt(2)],...
    'Color',fanuc.colors{7},'LineWidth',1.5);
set(L_t1,'Parent',hg);
L_t2 = line([l_t_rad,l_t_rad+l_t/sqrt(2)]*cos(3*pi/4),[l_t_rad,l_t_rad+l_t/sqrt(2)]*sin(3*pi/4),[0,l_t/sqrt(2)],...
    'Color',fanuc.colors{7},'LineWidth',1.5);
set(L_t2,'Parent',hg);
L_t3 = line([l_t_rad,l_t_rad+l_t/sqrt(2)]*cos(5*pi/4),[l_t_rad,l_t_rad+l_t/sqrt(2)]*sin(5*pi/4),[0,l_t/sqrt(2)],...
    'Color',fanuc.colors{7},'LineWidth',1.5);
set(L_t3,'Parent',hg);
L_t4 = line([l_t_rad,l_t_rad+l_t/sqrt(2)]*cos(7*pi/4),[l_t_rad,l_t_rad+l_t/sqrt(2)]*sin(7*pi/4),[0,l_t/sqrt(2)],...
    'Color',fanuc.colors{7},'LineWidth',1.5);
set(L_t4,'Parent',hg);
T_t = hgtransform('Parent',T_6,'Matrix',eye(4));
set(hg,'Parent',T_t);

% Tool brush frames
h = drawRobotFrame(fanuc.brush_colors{1});
hg = hggroup('parent',ax);
set(h,'Parent',hg);
T_7 = hgtransform('Parent',T_6,'Matrix',fanuc.tool{1});
set(hg,'Parent',T_7);
if fanuc.brush ~= 1
    set(T_7,'Visible','off');
end

h = drawRobotFrame(fanuc.brush_colors{2});
hg = hggroup('parent',ax);
set(h,'Parent',hg);
T_8 = hgtransform('Parent',T_6,'Matrix',fanuc.tool{2});
set(hg,'Parent',T_8);
if fanuc.brush ~= 2
    set(T_8,'Visible','off');
end

h = drawRobotFrame(fanuc.brush_colors{3});
hg = hggroup('parent',ax);
set(h,'Parent',hg);
T_9 = hgtransform('Parent',T_6,'Matrix',fanuc.tool{3});
set(hg,'Parent',T_9);
if fanuc.brush ~= 3
    set(T_9,'Visible','off');
end

h = drawRobotFrame(fanuc.brush_colors{4});
hg = hggroup('parent',ax);
set(h,'Parent',hg);
T_10 = hgtransform('Parent',T_6,'Matrix',fanuc.tool{4});
set(hg,'Parent',T_10);
if fanuc.brush ~= 4
    set(T_10,'Visible','off');
end

% Render graphics
set(gcf,'Renderer','openGL');
drawnow;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Return hgtransform handles
handles = [T_1,T_2,T_3,T_4,T_5,T_6,T_7,T_8,T_9,T_10];

    function h = drawRobotFrame( color )
         
        % Plot reference frame
        X_b = [vector_size,0,0,1]';
        Y_b = [0,vector_size,0,1]';
        Z_b = [0,0,vector_size,1]';
        h(1) = line(0,0,0,'Marker','.','MarkerSize',origin_size,'Color',color);
        h(2) = line([0,X_b(1)],[0,X_b(2)],[0,X_b(3)],'LineWidth',1.5,'Color',color);
        h(3) = line([0,Y_b(1)],[0,Y_b(2)],[0,Y_b(3)],'LineWidth',1.5,'Color',color);
        h(4) = line([0,Z_b(1)],[0,Z_b(2)],[0,Z_b(3)],'LineWidth',1.5,'Color',color);
        h(5) = line(X_b(1),X_b(2),X_b(3),'LineWidth',1.5,'Marker','x','MarkerSize',marker_size,'Color',color);
        h(6) = line(Y_b(1),Y_b(2),Y_b(3),'LineWidth',1.5,'Marker','o','MarkerSize',marker_size,'Color',color);
        h(7) = line(Z_b(1),Z_b(2),Z_b(3),'LineWidth',1.5,'Marker','d','MarkerSize',marker_size,'Color',color);
    end


end

