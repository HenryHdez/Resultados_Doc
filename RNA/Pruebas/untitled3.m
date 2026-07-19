%% Figura conjunta: Hmax, distancia y tiempo por morfologia y algoritmo
clear; clc; close all;

%% Etiquetas
algoritmos = {'Genetico', 'Ascenso colina', 'Propuesto', ...
              'Recocido', 'Caminata aleatoria'};

morfologias = {'Serpiente', 'Cuadrupedo', 'L', 'T'};

%% Datos promedio: Hmax
Hmean = [
    0.977  0.914  0.923  0.486  0.020;
    0.879  0.823  0.831  0.437  0.018;
    0.830  0.777  0.785  0.413  0.017;
    0.684  0.640  0.646  0.340  0.014
];

Hstd = [
    0.031  0.003  0.060  0.662  0.189;
    0.028  0.003  0.054  0.596  0.170;
    0.026  0.003  0.051  0.563  0.161;
    0.022  0.002  0.042  0.463  0.132
];

%% Datos promedio: distancia total D_T [m]
Dmean = [
    3.317  2.039  6.105  1.806  6.792;
    3.649  2.243  6.716  1.987  7.471;
    3.815  2.345  7.021  2.077  7.811;
    4.312  2.651  7.937  2.348  8.830
];

Dstd = [
    0.263  0.038  3.102  1.032  1.109;
    0.289  0.042  3.412  1.135  1.220;
    0.302  0.044  3.567  1.187  1.275;
    0.342  0.049  4.033  1.342  1.442
];

%% Datos promedio: tiempo de ejecucion T_e [s]
Tmean = [
    146.23  110.55  472.38  241.93  781.81;
    160.85  121.61  519.62  266.12  859.99;
    168.16  127.13  543.24  278.22  899.08;
    190.10  143.72  614.09  314.51 1016.35
];

Tstd = [
     46.29   6.31 264.36 10.58  81.89;
     50.92   6.94 290.80 11.64  90.08;
     53.23   7.26 304.01 12.17  94.17;
     60.18   8.20 343.67 13.75 106.46
];

%% Crear figura con tres subgraficas
figure('Color','w','Position',[100 100 1200 800]);

tiledlayout(3,1,'TileSpacing','compact','Padding','compact');

%% Subgrafica 1: Hmax
nexttile;
b1 = bar(Hmean,'grouped');
hold on;
agregarBarrasError(b1,Hmean,Hstd);
grid on; box on;
set(gca,'XTickLabel',morfologias,'FontSize',10);
ylabel('H_{max}');
title('(a) Mejor humedad detectada');
ylim([0 1.1]);

%% Subgrafica 2: Distancia total
nexttile;
b2 = bar(Dmean,'grouped');
hold on;
agregarBarrasError(b2,Dmean,Dstd);
grid on; box on;
set(gca,'XTickLabel',morfologias,'FontSize',10);
ylabel('D_T [m]');
title('(b) Distancia total recorrida');

%% Subgrafica 3: Tiempo de ejecucion
nexttile;
b3 = bar(Tmean,'grouped');
hold on;
agregarBarrasError(b3,Tmean,Tstd);
grid on; box on;
set(gca,'XTickLabel',morfologias,'FontSize',10);
xlabel('Morfologia del robot');
ylabel('T_e [s]');
title('(c) Tiempo de ejecucion');

%% Leyenda general
lgd = legend(algoritmos, ...
    'Orientation','horizontal', ...
    'Location','southoutside');

lgd.Layout.Tile = 'south';

%% Exportar figura
exportgraphics(gcf,'comparacion_algoritmos_morfologias.png','Resolution',300);

%% Funcion auxiliar para barras de error
function agregarBarrasError(b,Y,E)
    ngroups = size(Y,1);
    nbars = size(Y,2);

    x = nan(nbars,ngroups);

    for i = 1:nbars
        x(i,:) = b(i).XEndPoints;
    end

    errorbar(x',Y,E,'k', ...
        'linestyle','none', ...
        'linewidth',0.8, ...
        'CapSize',4);
end