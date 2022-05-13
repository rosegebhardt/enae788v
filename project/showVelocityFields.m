%% Uniform flow quiver plot

clearvars; close all; clc;

% Get the right colormap
coolwarm = getPyPlot_cMap('coolwarm',128);

% Define the grid used
xLim = 20; yLim = 20; gridSize = 40;
x = linspace(-xLim,xLim,gridSize); y = linspace(-yLim,yLim,gridSize);
[X,Y] = meshgrid(x,y);
Z = X + 1i*Y;

% Parameters
u0 = 1; % freestream velocity 

% Complex potential function
F = u0*Z;
Psi = imag(F); % Streamline function

% Conjugate complex velocity function
dFdz = u0*ones(size(Z));
U = real(dFdz); V = -imag(dFdz); % velocity components

% Plot velocity field
figure(1)
contourf(X,Y,Psi,50,'LineColor','none'); hold on;
velocity1 = quiver(X,Y,U,V,'k','Linewidth',1); hold on;
set(velocity1,'AutoScale','on','AutoScaleFactor',1);
colormap(coolwarm);
axis equal; xlim([-xLim,xLim]); ylim([-yLim,yLim]);
title('Uniform Horizontal Flow Field','interpreter','latex',...
    'fontsize',16)

%% Cylinder quiver plot

clearvars; close all; clc;

% Get the right colormap
coolwarm = getPyPlot_cMap('coolwarm',128);

% Define the grid used
xLim = 20; yLim = 20; gridSize = 40;
x = linspace(-xLim,xLim,gridSize); y = linspace(-yLim,yLim,gridSize);
[X,Y] = meshgrid(x,y);
Z = X + 1i*Y;

% Parameters
r0 = 8; % cylinder radius
u0 = 1; % freestream velocity 
Gamma = 0; % vorticity of cylinder

% Define cylinder position
theta = linspace(0,2*pi,1000);
circle = r0*exp(1i*theta);

% Complex potential function
F = u0*(Z + r0^2./Z) + 1j*Gamma*log(Z);
F(X.^2 + Y.^2 < r0^2) = 0;
Psi = imag(F); % Streamline function

% Conjugate complex velocity function
dFdz = u0*(1 - r0^2./(Z.^2));
U = real(dFdz); V = -imag(dFdz); % velocity components
U(X.^2 + Y.^2 < r0^2) = 0; V(X.^2 + Y.^2 < r0^2) = 0;

% Plot velocity field
figure(1)
contourf(X,Y,Psi,50,'LineColor','none'); hold on;
velocity1 = quiver(X,Y,U,V,'k','Linewidth',1); hold on;
set(velocity1,'AutoScale','on','AutoScaleFactor',1);
fill(real(circle),imag(circle),'k'); hold off;
colormap(coolwarm);
axis equal; xlim([-xLim,xLim]); ylim([-yLim,yLim]);
title('Cylinder in Freestream Velocity Field','interpreter','latex',...
    'fontsize',16)

%% Karman vortex street quiver plot

clearvars; close all; clc;

% Get the right colormap
coolwarm = getPyPlot_cMap('coolwarm', 128);

% Define the grid used
xLim = 6; yLim = 4; gridSize = 40;
x = linspace(-xLim,xLim,gridSize); y = linspace(-yLim,yLim,gridSize);
[X,Y] = meshgrid(x,y);
Z = X + 1i*Y;

% Parameters
zv = 0 - 0.75*1i; % first vortex location
gamma = 3; % vortex strength
h = 1.5; % vertical distance between vortices
a = 3; % horizontal distances between vertices

% Complex potential function
F = 1i*(gamma/(2*pi))*(log(sin(pi*(Z-zv)/a))-log(sin(pi*(Z-(a/2 + 1i*h)-zv)/a)));
Psi = imag(F); % Streamline function

% Conjugate complex velocity function
dFdz = (1i*gamma/(2*pi)) * (cot(pi*(Z-zv)/a) - cot(pi*(Z-zv-(a/2 + 1i*h))/a));
U = real(dFdz); V = -imag(dFdz); % velocity components

% Velocity at vortices approaches inf, cutoff values
cutoffVel = 4;

% Plot velocity field
figure(1)
contourf(X,Y,Psi,50,'LineColor','none'); hold on;
U(U.^2 + V.^2 > cutoffVel) = 0; V(U.^2 + V.^2 > cutoffVel) = 0;
velocity1 = quiver(X,Y,U,V,'k','Linewidth',1); hold on;
set(velocity1,'AutoScale','on','AutoScaleFactor',1.75);
colormap(coolwarm);
xlim([-xLim,xLim]); ylim([-yLim,yLim]); axis equal
title('Karman Vortex Street Velocity Field','interpreter','latex',...
    'fontsize',16)
