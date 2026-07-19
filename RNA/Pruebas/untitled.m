%% Representacion discreta del espacio de busqueda con cadena de modulos cuadrados
clear; clc; close all;

%% Dominio discreto Omega_d
Nx = 26;
Ny = 18;
[X,Y] = meshgrid(1:Nx,1:Ny);

%% Campo discreto de humedad mu(x)
mu = 0.35 ...
    + 0.35*exp(-((X-21).^2 + (Y-15).^2)/45) ...
    + 0.25*exp(-((X-15).^2 + (Y-4).^2)/30) ...
    - 0.20*exp(-((X-5).^2  + (Y-12).^2)/25);

mu = (mu - min(mu(:))) ./ (max(mu(:)) - min(mu(:)));

%% Figura
figure('Color','w','Position',[100 100 1200 750]);
hold on; axis equal; axis tight;

imagesc(mu);
set(gca,'YDir','normal');
colormap(parula);
alpha(0.78);

xlim([0.5 Nx+0.5]);
ylim([0.5 Ny+0.5]);

xlabel('$x_1$','Interpreter','latex','FontSize',18);
ylabel('$x_2$','Interpreter','latex','FontSize',18);
title('Espacio de busqueda $\Omega_d$', ...
    'Interpreter','latex','FontSize',18);

%% Grilla discreta
for i = 0.5:1:Nx+0.5
    plot([i i],[0.5 Ny+0.5],':','Color',[0.35 0.35 0.35]);
end

for j = 0.5:1:Ny+0.5
    plot([0.5 Nx+0.5],[j j],':','Color',[0.35 0.35 0.35]);
end

%% Obstaculo discreto
obs1 = [4 11; 5 11; 6 11; 4 12; 5 12; 6 12; 5 13; 6 13; 7 12];
drawDiscreteObstacle(obs1,'$\mathcal{O}_1$');

%% Trayectoria discreta x(t)
path = [ ...
    7 5;
    9 7;
    10 8;
    11 9;
    12 10;
    14 11;
    16 12;
    18 13;
    21 15];

plot(path(:,1),path(:,2),'k--','LineWidth',2.2);
plot(path(1,1),path(1,2),'ko','MarkerFaceColor','k','MarkerSize',8);

text(path(1,1)-1.2,path(1,2)-0.8,'$x(0)$', ...
    'Interpreter','latex','FontSize',15);

quiver(path(end-1,1),path(end-1,2), ...
    path(end,1)-path(end-1,1), ...
    path(end,2)-path(end-1,2), ...
    0,'k','LineWidth',2.2,'MaxHeadSize',0.8);

text(21.4,15.3,'$x(t)$', ...
    'Interpreter','latex','FontSize',16);

%% Cadena tipo serpiente con cuatro modulos cuadrados
Rs = 1.2;

snake = [ ...
    10.8 8.8;
    11.6 9.4;
    12.5 9.0;
    13.3 9.6];

%% Conexion fisica entre modulos
plot(snake(:,1),snake(:,2),'k-','LineWidth',4.0);

for n = 1:size(snake,1)

    xi = snake(n,1);
    yi = snake(n,2);

    %% Rango de sensado
    th = linspace(0,2*pi,160);
    plot(xi + Rs*cos(th), yi + Rs*sin(th), '--', ...
        'Color',[0 0.45 0.9], 'LineWidth',1.2);

    %% Modulo cuadrado
    rectangle('Position',[xi-0.35 yi-0.35 0.7 0.7], ...
        'Curvature',0.15, ...
        'FaceColor','w', ...
        'EdgeColor','k', ...
        'LineWidth',1.8);

    %% Punto central del modulo
    plot(xi,yi,'ko','MarkerFaceColor','k','MarkerSize',3);

    %% Etiqueta del modulo
    text(xi-0.25,yi-0.85,sprintf('$m_%d$',n), ...
        'Interpreter','latex','FontSize',13);

    %% Direcciones candidatas Q
    dirs = [ ...
         1  0;
        -1  0;
         0  1;
         0 -1];

    for d = 1:size(dirs,1)
        v = dirs(d,:) / norm(dirs(d,:));
        quiver(xi,yi,0.65*v(1),0.65*v(2),0, ...
            'Color',[0.45 0.45 0.45], ...
            'LineStyle','--', ...
            'LineWidth',0.9, ...
            'MaxHeadSize',0.8);
    end

    %% Direccion deseada d*
    [gx,gy] = localDiscreteGradient(mu,xi,yi);
    gnorm = hypot(gx,gy) + eps;

    quiver(xi,yi,1.05*gx/gnorm,1.05*gy/gnorm,0, ...
        'Color',[0 0.45 0.12], ...
        'LineWidth',2.3, ...
        'MaxHeadSize',0.9);
end

%% Comunicacion local entre modulos vecinos
for n = 1:size(snake,1)-1
    plot(snake(n:n+1,1),snake(n:n+1,2),'k:', ...
        'LineWidth',2.4);
end

%% Etiquetas principales
text(19.2,16.6,'$\mu(\mathbf{x})$', ...
    'Interpreter','latex','FontSize',15,'FontWeight','bold');

text(14.8,12.8,'$\mathbf{d}^{\star}(\mathbf{x},k)$', ...
    'Interpreter','latex','FontSize',14, ...
    'Color',[0 0.35 0.1],'FontWeight','bold');

text(9.0,9.2,'$R_s$', ...
    'Interpreter','latex','FontSize',14, ...
    'Color',[0 0.25 0.8],'FontWeight','bold');

%% Barra de color
cb = colorbar;
cb.Label.String = 'Humedad del suelo \mu(x) - VWC';
cb.Label.FontSize = 12;

%% Ajustes finales
set(gca,'FontSize',12);
set(gca,'Layer','top');

%% Exportar imagen
print(gcf,'representacion_discreta_serpiente_cuadrada','-dpng','-r300');

%% Funciones locales

function drawDiscreteObstacle(cells,labelText)

    for r = 1:size(cells,1)
        x = cells(r,1);
        y = cells(r,2);

        rectangle('Position',[x-0.5 y-0.5 1 1], ...
            'FaceColor',[0.45 0.45 0.45], ...
            'EdgeColor',[0.2 0.2 0.2], ...
            'LineWidth',1.0);
    end

    cx = mean(cells(:,1));
    cy = mean(cells(:,2));

    text(cx+1.3,cy+1,labelText, ...
        'Interpreter','latex', ...
        'FontSize',15, ...
        'Color',[0.65 0 0], ...
        'FontWeight','bold');

    th = linspace(0,2*pi,160);
    rx = max(cells(:,1))-min(cells(:,1)) + 1.8;
    ry = max(cells(:,2))-min(cells(:,2)) + 1.8;

    plot(cx + 0.55*rx*cos(th), ...
         cy + 0.55*ry*sin(th), ...
         'r--','LineWidth',1.4);
end

function [gx,gy] = localDiscreteGradient(mu,xi,yi)

    [Ny,Nx] = size(mu);

    xi = round(xi);
    yi = round(yi);

    xi = max(2,min(Nx-1,xi));
    yi = max(2,min(Ny-1,yi));

    gx = (mu(yi,xi+1) - mu(yi,xi-1))/2;
    gy = (mu(yi+1,xi) - mu(yi-1,xi))/2;
end