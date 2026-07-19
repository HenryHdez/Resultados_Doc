%% Potencial ambiental discreto con fuente de humedad y obstáculos grises 2x2
clear; clc; close all;

%% Dominio discreto
Nx = 26;
Ny = 18;

[X,Y] = meshgrid(1:Nx,1:Ny);

%% Fuente de humedad mu(x)
mu = 0.20 ...
    + 0.80*exp(-((X-21).^2 + (Y-14).^2)/35);

mu = (mu - min(mu(:))) ./ (max(mu(:)) - min(mu(:)));

%% Obstáculos discretos 2x2
Obs = zeros(Ny,Nx);

Obs(8:9,9:10)    = 1;   % Obstáculo 1
Obs(5:6,15:16)   = 1;   % Obstáculo 2
Obs(11:12,17:18) = 1;   % Obstáculo 3

%% Campo repulsivo rho(x)
rho = zeros(size(X));

rho = rho + exp(-((X-9.5).^2  + (Y-8.5).^2)/5);
rho = rho + exp(-((X-15.5).^2 + (Y-5.5).^2)/5);
rho = rho + exp(-((X-17.5).^2 + (Y-11.5).^2)/5);

rho = rho ./ max(rho(:));

%% Potencial ambiental Gamma
kappa_mu = 1.2;
kappa_o  = 0.7;

Gamma = kappa_mu*mu - kappa_o*rho;

%% Normalizar Gamma entre 0 y 1
Gamma = (Gamma - min(Gamma(:))) ./ ...
        (max(Gamma(:)) - min(Gamma(:)));

%% Gradiente discreto
[dGdy,dGdx] = gradient(Gamma);

normG = sqrt(dGdx.^2 + dGdy.^2) + 1e-6;

Ux = dGdx ./ normG;
Uy = dGdy ./ normG;

%% Figura única
figure('Color','w','Position',[100 100 900 600]);

imagesc(Gamma);
set(gca,'YDir','normal');
caxis([0 1]);
hold on;

%% Rejilla discreta
for i = 1:Nx
    xline(i,'Color',[0.75 0.75 0.75],'LineWidth',0.3);
end

for j = 1:Ny
    yline(j,'Color',[0.75 0.75 0.75],'LineWidth',0.3);
end

%% Dibujar obstáculos grises
for r = 1:Ny
    for c = 1:Nx
        if Obs(r,c) == 1
            rectangle( ...
                'Position',[c-0.5,r-0.5,1,1], ...
                'FaceColor',[0.50 0.50 0.50], ...
                'EdgeColor',[0.20 0.20 0.20], ...
                'LineWidth',1.1);
        end
    end
end

%% Fuente de humedad
contour(X,Y,mu,[0.75 0.90],'w','LineWidth',2);

text(20.2,14.8,'Fuente de humedad', ...
    'Color','w', ...
    'FontWeight','bold', ...
    'FontSize',11);

%% Flechas direccionales discretas
step = 2;

quiver(X(1:step:end,1:step:end), ...
       Y(1:step:end,1:step:end), ...
       Ux(1:step:end,1:step:end), ...
       Uy(1:step:end,1:step:end), ...
       0.45, ...
       'k', ...
       'LineWidth',1);

%% Etiquetas de obstáculos
text(8.8,9.6,'Obs. 1','Color','k','FontWeight','bold');
text(14.8,6.6,'Obs. 2','Color','k','FontWeight','bold');
text(16.8,12.6,'Obs. 3','Color','k','FontWeight','bold');

%% Presentación
axis equal tight;
xlim([1 Nx]);
ylim([1 Ny]);

xlabel('x');
ylabel('y');

title({'Campo potencial discreto', ...
       '\Gamma(x,k)=\kappa_\mu\hat{\mu}(x,k)-\kappa_o\rho(x,k)'});

cb = colorbar;
ylabel(cb,'Potencial ambiental \Gamma');

colormap parula;
box on;