%%
thetas = [pi/3,2*pi/3,pi/2;pi/3,2*pi/3,pi/2;pi/3,2*pi/3,pi/2];

% work space of the Novint Falcon [ xmin, xmax, ymin, ymax, zmin, zmax]
workspace = [-60,60,-60,60,0,200];
NF_T = NovintFalcon_FK( thetas );

% Create figure window
figure('Color','w');

% Create axes object
ax = axes('XLim',workspace(1:2),'YLim',workspace(3:4),'ZLim',workspace(5:6));
% vw = [31.3,22.8];
set(gca,'View',vw);
grid minor;
axis equal;
xlabel('X (mm)','FontSize',10);
ylabel('Y (mm)','FontSize',10);
zlabel('Z (mm)','FontSize',10);
%
%%

