
%% Figura integrada: espacio de busqueda, humedad, obstaculos y robot modular
% Autor: Adaptado para articulo de robot modular y monitoreo de humedad
% Descripcion:
% Genera una figura tipo paper con:
% - Campo discreto de humedad del suelo
% - Campo vectorial de navegacion
% - Fuente de humedad
% - Obstaculos
% - Robot modular tipo cadena
% - Radios de sensado
% - Trayectoria x(0) -> x(t)
% - Direccion deseada d*(x,k)

clear; clc; close all;

%% ============================================================
% 1. Dominio discreto de busqueda
% ============================================================

Nx = 26;              % Numero de celdas en x
Ny = 18;              % Numero de celdas en y

x = 1:Nx;
y = 1:Ny;

[X,Y] = meshgrid(x,y);

%% ============================================================
% 2. Campo de humedad del suelo mu(x)
% ============================================================

% Campo base con fuente principal de humedad y variaciones espaciales
mu = 0.35 ...
    + 0.55*exp(-((X-22).^2 + (Y-15).^2)/38) ...   % Fuente principal
    + 0.25*exp(-((X-15).^2 + (Y-4).^2)/45) ...    % Humedad secundaria
    - 0.22*exp(-((X-6).^2  + (Y-11).^2)/18);      % Zona de baja humedad

% Normalizacion entre 0 y 1
mu = (mu - min(mu(:))) ./ (max(mu(:)) - min(mu(:)));

%% ============================================================
% 3. Obstaculos
% ============================================================

obs = false(Ny,Nx);

% Obstaculo 1: bloque irregular izquierdo
obs(10:12,4:7) = true;
obs(13,5:6)    = true;
obs(11,8)      = true;

% Obstaculo 2: bloque inferior central
obs(5:6,15:16) = true;

% Obstaculo 3: bloque superior central
obs(12:13,16:17) = true;

% Penalizacion del campo por obstaculos
mu_obs = mu;
mu_obs(obs) = NaN;

%% ============================================================
% 4. Campo vectorial de navegacion
% ============================================================

% Gradiente del campo de humedad
[dmu_dy,dmu_dx] = gradient(mu);

% Campo repulsivo alrededor de obstaculos
Ux_rep = zeros(size(mu));
Uy_rep = zeros(size(mu));

[oy, ox] = find(obs);

for n = 1:length(ox)
    dx = X - ox(n);
    dy = Y - oy(n);
    r2 = dx.^2 + dy.^2 + 0.5;
    
    % Repulsion decreciente con la distancia
    influence = exp(-r2/5);
    Ux_rep = Ux_rep + influence .* dx ./ sqrt(r2);
    Uy_rep = Uy_rep + influence .* dy ./ sqrt(r2);
end

% Campo combinado: atraccion por humedad + repulsion por obstaculos
k_mu = 1.0;
k_obs = 0.65;

U = k_mu*dmu_dx + k_obs*Ux_rep;
V = k_mu*dmu_dy + k_obs*Uy_rep;

% Normalizar vectores para visualizacion
mag = sqrt(U.^2 + V.^2);
Uq = U ./ (mag + eps);
Vq = V ./ (mag + eps);

% Submuestreo del campo vectorial
step = 2;
Xq = X(1:step:end,1:step:end);
Yq = Y(1:step:end,1:step:end);
Uq = Uq(1:step:end,1:step:end);
Vq = Vq(1:step:end,1:step:end);

%% ============================================================
% 5. Crear figura
% ============================================================

figure('Color','w','Position',[100 80 1200 850]);
hold on;

% Mapa de humedad
imagesc(x,y,mu);
set(gca,'YDir','normal');

% Mapa de color
colormap(turbo);
caxis([0 1]);

% Grilla
xticks(0:1:Nx);
yticks(0:1:Ny);
grid on;
set(gca,'GridColor',[1 1 1],'GridAlpha',0.35,'LineWidth',1.0);

% Ajuste visual de ejes
xlim([0 Nx+1]);
ylim([0 Ny+1]);
axis equal tight;

%% ============================================================
% 6. Dibujar obstaculos
% ============================================================

drawGridObstacle(obs, [0.45 0.45 0.45]);

% Etiquetas de obstaculos
text(6.6,12.2,'$\Omega_1$', ...
    'Interpreter','latex', ...
    'Color','r', ...
    'FontSize',22, ...
    'FontWeight','bold');

text(15.0,6.7,'Obs. 2', ...
    'Interpreter','latex', ...
    'Color','k', ...
    'FontSize',13, ...
    'FontWeight','bold');

text(16.0,13.5,'Obs. 3', ...
    'Interpreter','latex', ...
    'Color','k', ...
    'FontSize',13, ...
    'FontWeight','bold');

% Region de influencia del obstaculo 1
theta = linspace(0,2*pi,300);
cx = 5.8; cy = 11.3;
rx = 3.4; ry = 1.9;

plot(cx + rx*cos(theta), cy + ry*sin(theta), ...
    'r--','LineWidth',2.0);

%% ============================================================
% 7. Fuente de humedad y contornos
% ============================================================

source_x = 22;
source_y = 15;

% Contornos blancos alrededor de la fuente
contour(X,Y,mu,[0.72 0.82 0.92], ...
    'LineColor','w', ...
    'LineWidth',2.5);

text(source_x-1.1,source_y+0.1, ...
    {'Moisture','source'}, ...
    'Color','k', ...
    'FontSize',14, ...
    'FontWeight','bold', ...
    'HorizontalAlignment','center');

%% ============================================================
% 8. Campo vectorial
% ============================================================

quiver(Xq,Yq,Uq,Vq,0.7, ...
    'k', ...
    'LineWidth',1.4, ...
    'MaxHeadSize',1.8);

%% ============================================================
% 9. Trayectoria del robot
% ============================================================

% Puntos de trayectoria conceptual
path_x = [7 8.5 10.0 11.5 13.0 14.5 16.0 18.0 20.0 22.0];
path_y = [4.5 6.0 7.4 8.3 8.8 9.2 10.0 11.2 13.0 15.0];

% Trayectoria punteada
plot(path_x,path_y,'k--','LineWidth',3.0);

% Punto inicial
plot(path_x(1),path_y(1),'ko', ...
    'MarkerFaceColor','k', ...
    'MarkerSize',10);

text(path_x(1)-0.7,path_y(1)-0.9,'$\mathbf{x}(0)$', ...
    'Interpreter','latex', ...
    'FontSize',18, ...
    'Color','k', ...
    'FontWeight','bold');

% Flecha final hacia fuente
annotation_arrow(gca,[20.2 22.1],[12.4 14.9], ...
    'Color','k','LineWidth',4);

text(22.3,14.2,'$\mathbf{x}(t)$', ...
    'Interpreter','latex', ...
    'FontSize',18, ...
    'Color','k', ...
    'FontWeight','bold');

% Direccion deseada
text(15.7,10.7,'$\mathbf{d}^{*}(\mathbf{x},k)$', ...
    'Interpreter','latex', ...
    'FontSize',18, ...
    'Color','k', ...
    'FontWeight','bold');

%% ============================================================
% 10. Robot modular tipo cadena
% ============================================================

% Posiciones de los modulos
robot_x = [10.2 11.4 12.6 14.0];
robot_y = [7.1 8.0 8.4 8.9];

% Dibujar radios de sensado
Rs = 1.25;
for i = 1:length(robot_x)
    plot(robot_x(i) + Rs*cos(theta), ...
         robot_y(i) + Rs*sin(theta), ...
         'b--','LineWidth',1.5);
end

% Dibujar enlaces entre modulos
plot(robot_x,robot_y,'k-','LineWidth',4);

% Dibujar modulos
module_size = 0.65;
for i = 1:length(robot_x)
    rectangle('Position',[robot_x(i)-module_size/2, ...
                          robot_y(i)-module_size/2, ...
                          module_size, module_size], ...
              'Curvature',0.15, ...
              'FaceColor','w', ...
              'EdgeColor','k', ...
              'LineWidth',2.5);
    
    % Etiqueta del modulo
    text(robot_x(i)-0.25,robot_y(i)-0.85, ...
        ['$m_',num2str(i),'$'], ...
        'Interpreter','latex', ...
        'FontSize',15, ...
        'FontWeight','bold', ...
        'Color','k');
    
    % Flecha verde de orientacion local
    annotation_arrow(gca, ...
        [robot_x(i)-0.15 robot_x(i)+0.55], ...
        [robot_y(i)-0.05 robot_y(i)+0.45], ...
        'Color',[0 0.45 0], ...
        'LineWidth',2.5);
end

%% ============================================================
% 11. Etiquetas, titulo y barra de color
% ============================================================

title('Search space $\Omega_d$', ...
    'Interpreter','latex', ...
    'FontSize',24, ...
    'FontWeight','bold');

xlabel('$x_1$', ...
    'Interpreter','latex', ...
    'FontSize',20);

ylabel('$x_2$', ...
    'Interpreter','latex', ...
    'FontSize',20);

cb = colorbar;
cb.Label.String = 'Soil moisture $\mu(\mathbf{x})$ - VWC';
cb.Label.Interpreter = 'latex';
cb.Label.FontSize = 18;
cb.Ticks = 0:0.2:1;
cb.FontSize = 13;

% Mejorar aspecto del eje
set(gca, ...
    'FontSize',14, ...
    'TickLabelInterpreter','latex', ...
    'Box','on', ...
    'Layer','top');

%% ============================================================
% 12. Exportar figura
% ============================================================

exportgraphics(gcf,'search_space_modular_robot.png','Resolution',300);
exportgraphics(gcf,'search_space_modular_robot.pdf','ContentType','vector');

disp('Figura exportada como: search_space_modular_robot.png y search_space_modular_robot.pdf');

%% ============================================================
% FUNCIONES AUXILIARES
% ============================================================

function drawGridObstacle(obs, faceColor)
    % Dibuja obstaculos celda por celda sobre una grilla
    [rows, cols] = find(obs);
    
    for i = 1:length(rows)
        x0 = cols(i) - 0.5;
        y0 = rows(i) - 0.5;
        
        rectangle('Position',[x0 y0 1 1], ...
            'FaceColor',faceColor, ...
            'EdgeColor',[0.15 0.15 0.15], ...
            'LineWidth',1.0);
    end
end

function annotation_arrow(ax, xdata, ydata, varargin)
    % Dibuja una flecha dentro de un eje usando annotation,
    % convirtiendo coordenadas de datos a coordenadas normalizadas.
    
    fig = ancestor(ax,'figure');
    
    % Guardar unidades originales
    oldUnitsAx = ax.Units;
    oldUnitsFig = fig.Units;
    
    ax.Units = 'normalized';
    fig.Units = 'normalized';
    
    axpos = ax.Position;
    xlim_ax = ax.XLim;
    ylim_ax = ax.YLim;
    
    % Conversion a coordenadas normalizadas de figura
    xnorm = axpos(1) + (xdata - xlim_ax(1)) ./ diff(xlim_ax) * axpos(3);
    ynorm = axpos(2) + (ydata - ylim_ax(1)) ./ diff(ylim_ax) * axpos(4);
    
    annotation(fig,'arrow',xnorm,ynorm,varargin{:});
    
    % Restaurar unidades
    ax.Units = oldUnitsAx;
    fig.Units = oldUnitsFig;
end
