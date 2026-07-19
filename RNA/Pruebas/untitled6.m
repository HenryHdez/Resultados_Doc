%% Boxplots de metricas de navegacion
% Datos reconstruidos a partir de media y desviacion estandar
% Controladores: SMG-DN, Hill Climbing, ACO, GA

clear; clc; close all;

%% Configuracion
rng(10);                 % Semilla para reproducibilidad
N = 90;                  % 3 condiciones x 3 configuraciones x 10 repeticiones
controllers = {'SMG-DN','Hill Climbing','ACO','Genetic Algorithm'};

%% Medias y desviaciones estandar por controlador
% Orden: SMG-DN, Hill Climbing, ACO, GA

tc_mean = [18.6, 21.4, 23.8, 25.4];
tc_std  = [1.4,  1.5,  1.8,  1.7];

err_mean = [0.23, 0.28, 0.33, 0.30];
err_std  = [0.02, 0.02, 0.03, 0.02];

eta_mean = [0.91, 0.89, 0.87, 0.85];
eta_std  = [0.01, 0.01, 0.01, 0.01];

varv_mean = [0.022, 0.027, 0.032, 0.029];
varv_std  = [0.002, 0.002, 0.002, 0.002];

%% Generacion de datos aproximados
tc_data   = generarDatos(tc_mean, tc_std, N);
err_data  = generarDatos(err_mean, err_std, N);
eta_data  = generarDatos(eta_mean, eta_std, N);
varv_data = generarDatos(varv_mean, varv_std, N);

%% Figura
figure('Color','w','Position',[100 100 900 650]);

subplot(2,2,1);
boxplot(tc_data, 'Labels', controllers);
ylabel('Value');
title('Convergence Time (s)');
grid on;
set(gca,'FontSize',9);
xtickangle(20);

subplot(2,2,2);
boxplot(err_data, 'Labels', controllers);
ylabel('Value');
title('Angular Error (rad)');
grid on;
set(gca,'FontSize',9);
xtickangle(20);

subplot(2,2,3);
boxplot(eta_data, 'Labels', controllers);
ylabel('Value');
title('Efficiency (\eta)');
grid on;
set(gca,'FontSize',9);
xtickangle(20);

subplot(2,2,4);
boxplot(varv_data, 'Labels', controllers);
ylabel('Value');
title('Velocity Variance');
grid on;
set(gca,'FontSize',9);
xtickangle(20);

%% Exportar figura
exportgraphics(gcf, 'boxplot_navigation_metrics.png', 'Resolution', 300);
exportgraphics(gcf, 'boxplot_navigation_metrics.pdf', 'ContentType', 'vector');

%% Funcion auxiliar
function data = generarDatos(mu, sigma, N)
    nMethods = numel(mu);
    data = zeros(N, nMethods);

    for i = 1:nMethods
        x = mu(i) + sigma(i).*randn(N,1);

        % Evitar valores no fisicos
        if mu(i) < 1
            x(x < 0) = mu(i) - 0.5*sigma(i);
        end

        data(:,i) = x;
    end
end