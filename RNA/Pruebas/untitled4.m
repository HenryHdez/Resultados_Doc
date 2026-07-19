%% Mapa de calor normalizado para comparar algoritmos en configuración serpiente
clear; clc; close all;

%% Datos originales
algoritmos = {'Frontera', 'Gradiente local', 'Algoritmo genetico', 'Propuesto'};

Hmax = [0.714, 0.853, 1.000, 0.947];
Hf   = [0.220, 0.349, 0.287, 0.629];
DT   = [14.614, 14.784, 14.205, 14.669];
etheta = [0.611, 0.801, 0.435, 0.691];
Md   = [45.238, 57.887, 60.417, 45.437];

%% Normalización
% Para Hmax, Hf y Md: mayor es mejor.
% Para DT y error angular: menor es mejor.

Hmax_n = normalizarMayorMejor(Hmax);
Hf_n   = normalizarMayorMejor(Hf);
Md_n   = normalizarMayorMejor(Md);

DT_n      = normalizarMenorMejor(DT);
etheta_n  = normalizarMenorMejor(etheta);

%% Matriz normalizada
M = [Hmax_n(:), Hf_n(:), DT_n(:), etheta_n(:), Md_n(:)];

metricas = {'H_{max}', 'H_f', 'D_T', 'e_\theta', 'M_d'};

%% Puntaje global promedio
score = mean(M, 2);

%% Figura 1: mapa de calor
figure('Color','w','Position',[100 100 900 420]);

h = heatmap(metricas, algoritmos, M);

h.Title = 'Comparación normalizada de algoritmos - configuración serpiente';
h.XLabel = 'Métricas normalizadas';
h.YLabel = 'Algoritmos';

% Escala de color
colormap parula;
h.ColorLimits = [0 1];

%% Exportar figura
exportgraphics(gcf, 'heatmap_algoritmos_serpiente.png', 'Resolution', 300);

%% Figura 2: puntaje global
figure('Color','w','Position',[100 100 750 420]);

bar(score);
grid on;
box on;

set(gca, 'XTickLabel', algoritmos, 'XTickLabelRotation', 25, 'FontSize', 11);
ylabel('Puntaje global normalizado');
title('Puntaje global de desempeño por algoritmo');

ylim([0 1]);

%% Agregar valores sobre las barras
for i = 1:length(score)
    text(i, score(i) + 0.03, sprintf('%.2f', score(i)), ...
        'HorizontalAlignment', 'center', ...
        'FontSize', 10);
end

%% Exportar figura
exportgraphics(gcf, 'score_algoritmos_serpiente.png', 'Resolution', 300);

%% Funciones auxiliares
function y = normalizarMayorMejor(x)
    if max(x) == min(x)
        y = ones(size(x));
    else
        y = (x - min(x)) ./ (max(x) - min(x));
    end
end

function y = normalizarMenorMejor(x)
    if max(x) == min(x)
        y = ones(size(x));
    else
        y = (max(x) - x) ./ (max(x) - min(x));
    end
end