function [ ~ ] = drawNovintFalcon( thetas )

% work space of the Novint Falcon [ xmin, xmax, ymin, ymax, zmin, zmax]
workspace = [-60,60,-60,60,0,200];
NF_T = NovintFalcon_FK( thetas );

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


end

