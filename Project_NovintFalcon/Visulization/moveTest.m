clear; close all;

% Creates a 2D Mesh to plot surface
x=linspace(0,1,100);
[X,Y] = meshgrid(x,x);

v = VideoWriter('WaveMovie.avi');
open(v);

N=100; % Number of frames
for i = 1:N
    % Example of plot 
    Z = sin(2*pi*(X-i/N)).*sin(2*pi*(Y-i/N));
    surf(X,Y,Z)
    
    % Store the frame   
    frame = getframe(gcf); % leaving gcf out crops the frame in the movie. 
    writeVideo(v,frame);
end 

close(v);
