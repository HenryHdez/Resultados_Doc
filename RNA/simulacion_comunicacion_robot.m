%% SIMULACION_COMUNICACION_ROBOT
% Simula la comunicacion cliente-servidor de un robot modular.
% Cada modulo actua como cliente y envia telemetria al servidor central.
% El servidor procesa los datos y responde con una referencia de control.
%
% El modelo genera:
%   - paquetes transmitidos, recibidos y perdidos;
%   - latencia de subida, procesamiento, bajada y tiempo de ida y vuelta;
%   - jitter, tasa de actualizacion y edad de la informacion (AoI);
%   - registros CSV y figuras para analizar los resultados.
%
% No requiere hardware ni sockets UDP reales. Es una simulacion de eventos
% discretos que permite definir condiciones reproducibles del canal.

clear; clc; close all;
rng(42);                         % Reproducibilidad

%% 1. Configuracion general
cfg.nModulos       = 6;
cfg.duracion_s     = 120;
cfg.Tenvio_s       = 0.050;      % Periodo nominal: 50 ms
cfg.jitterReloj_s  = 0.004;      % Variacion del instante de generacion
cfg.timeout_s      = 0.200;

% Canal cliente -> servidor
cfg.perdidaUp      = 0.025;      % Probabilidad de perdida (2.5 %)
cfg.retardoUp_s    = 0.018;      % Retardo medio
cfg.sigmaUp_s      = 0.006;

% Procesamiento en el servidor
cfg.procMedio_s    = 0.008;
cfg.procSigma_s    = 0.002;

% Canal servidor -> cliente
cfg.perdidaDown    = 0.015;
cfg.retardoDown_s  = 0.015;
cfg.sigmaDown_s    = 0.005;

% Rafaga de degradacion para representar interferencia o congestion
cfg.usarRafaga     = true;
cfg.inicioRafaga_s = 45;
cfg.finRafaga_s    = 65;
cfg.perdidaRafaga  = 0.18;
cfg.retardoExtra_s = 0.060;

%% 2. Generacion de eventos de telemetria
tNominal = (0:cfg.Tenvio_s:cfg.duracion_s-cfg.Tenvio_s)';
nPorModulo = numel(tNominal);
nTotal = cfg.nModulos*nPorModulo;

modulo        = zeros(nTotal,1);
secuencia     = zeros(nTotal,1);
tGeneracion   = zeros(nTotal,1);
humedad       = zeros(nTotal,1);
proxFrente    = zeros(nTotal,1);
proxIzquierda = zeros(nTotal,1);
proxDerecha   = zeros(nTotal,1);

fila = 0;
for m = 1:cfg.nModulos
    idx = fila + (1:nPorModulo);
    tg = tNominal + cfg.jitterReloj_s*randn(nPorModulo,1);
    tg = max(tg,0);

    modulo(idx)    = m;
    secuencia(idx) = (1:nPorModulo)';
    tGeneracion(idx) = tg;

    % Variables sensoriales sinteticas, acotadas en [0,1]
    humedad(idx) = sat01(0.40 + 0.18*sin(2*pi*tg/35 + 0.25*m) ...
                       + 0.035*randn(nPorModulo,1));
    proxFrente(idx) = sat01(0.15 + 0.75*exp(-((tg-55)/5).^2) ...
                          + 0.03*randn(nPorModulo,1));
    proxIzquierda(idx) = sat01(0.12 + 0.55*exp(-((tg-82)/7).^2) ...
                              + 0.03*randn(nPorModulo,1));
    proxDerecha(idx) = sat01(0.10 + 0.50*exp(-((tg-28)/6).^2) ...
                            + 0.03*randn(nPorModulo,1));
    fila = fila + nPorModulo;
end

%% 3. Simulacion del enlace cliente -> servidor
enRafaga = cfg.usarRafaga & tGeneracion >= cfg.inicioRafaga_s ...
                         & tGeneracion <= cfg.finRafaga_s;

pUp = cfg.perdidaUp*ones(nTotal,1);
pUp(enRafaga) = cfg.perdidaRafaga;
perdidoUp = rand(nTotal,1) < pUp;

latUp = max(0.001, cfg.retardoUp_s + cfg.sigmaUp_s*randn(nTotal,1));
latUp(enRafaga) = latUp(enRafaga) + cfg.retardoExtra_s.*rand(sum(enRafaga),1);
tRecepcionServidor = tGeneracion + latUp;
tRecepcionServidor(perdidoUp) = NaN;

%% 4. Procesamiento del servidor y respuesta de control
tProcesamiento = max(0.001, cfg.procMedio_s + cfg.procSigma_s*randn(nTotal,1));
tRespuestaServidor = tRecepcionServidor + tProcesamiento;

% Regla ilustrativa del servidor: 1 avanzar, 2 izquierda, 3 derecha, 4 evasion
accion = nan(nTotal,1);
recibidosUp = ~perdidoUp;
accion(recibidosUp) = 1;
accion(recibidosUp & proxIzquierda > proxDerecha & proxIzquierda > 0.55) = 3;
accion(recibidosUp & proxDerecha > proxIzquierda & proxDerecha > 0.55) = 2;
accion(recibidosUp & proxFrente >= 0.85) = 4;

%% 5. Simulacion del enlace servidor -> cliente
perdidoDown = false(nTotal,1);
pDown = cfg.perdidaDown*ones(nTotal,1);
pDown(enRafaga) = min(1,0.75*cfg.perdidaRafaga);
perdidoDown(recibidosUp) = rand(sum(recibidosUp),1) < pDown(recibidosUp);

latDown = max(0.001, cfg.retardoDown_s + cfg.sigmaDown_s*randn(nTotal,1));
latDown(enRafaga) = latDown(enRafaga) + 0.7*cfg.retardoExtra_s.*rand(sum(enRafaga),1);

tRecepcionCliente = tRespuestaServidor + latDown;
tRecepcionCliente(perdidoUp | perdidoDown) = NaN;

entregaCompleta = ~isnan(tRecepcionCliente);
latenciaUp = tRecepcionServidor - tGeneracion;
latenciaDown = tRecepcionCliente - tRespuestaServidor;
RTT = tRecepcionCliente - tGeneracion;
AoI = tRecepcionServidor - tGeneracion;  % Edad al llegar al servidor
timeout = ~entregaCompleta | RTT > cfg.timeout_s;

%% 6. Registro completo de paquetes
estado = strings(nTotal,1);
estado(perdidoUp) = "Perdido_subida";
estado(~perdidoUp & perdidoDown) = "Perdido_bajada";
estado(entregaCompleta & ~timeout) = "Entregado";
estado(entregaCompleta & timeout) = "Entregado_fuera_timeout";

registro = table(modulo,secuencia,tGeneracion,humedad,proxFrente, ...
    proxIzquierda,proxDerecha,enRafaga,perdidoUp,tRecepcionServidor, ...
    latenciaUp,tProcesamiento,accion,perdidoDown,tRecepcionCliente, ...
    latenciaDown,RTT,AoI,timeout,estado);

%% 7. Metricas por modulo
resumen = table('Size',[cfg.nModulos 12], ...
    'VariableTypes',repmat("double",1,12), ...
    'VariableNames',{'Modulo','Ntx','NrxServidor','NrxCliente', ...
    'PerdidaUp_pct','PerdidaFinFin_pct','LatenciaUp_ms','RTT_ms', ...
    'JitterRTT_ms','AoI_ms','Tactualizacion_ms','Factualizacion_Hz'});

for m = 1:cfg.nModulos
    im = modulo == m;
    okUp = im & ~isnan(tRecepcionServidor);
    okFin = im & entregaCompleta;
    tiemposRx = sort(tRecepcionServidor(okUp));
    interRx = diff(tiemposRx);

    resumen.Modulo(m) = m;
    resumen.Ntx(m) = sum(im);
    resumen.NrxServidor(m) = sum(okUp);
    resumen.NrxCliente(m) = sum(okFin);
    resumen.PerdidaUp_pct(m) = 100*(1-sum(okUp)/sum(im));
    resumen.PerdidaFinFin_pct(m) = 100*(1-sum(okFin)/sum(im));
    resumen.LatenciaUp_ms(m) = 1000*mean(latenciaUp(okUp),'omitnan');
    resumen.RTT_ms(m) = 1000*mean(RTT(okFin),'omitnan');
    resumen.JitterRTT_ms(m) = 1000*std(diff(RTT(okFin)),'omitnan');
    resumen.AoI_ms(m) = 1000*mean(AoI(okUp),'omitnan');
    resumen.Tactualizacion_ms(m) = 1000*mean(interRx,'omitnan');
    resumen.Factualizacion_Hz(m) = 1/mean(interRx,'omitnan');
end

%% 8. Resumen global
okUp = ~isnan(tRecepcionServidor);
okFin = entregaCompleta;
tiemposGlobales = sort(tRecepcionServidor(okUp));
interGlobal = diff(tiemposGlobales);

globalMetricas = table( ...
    nTotal,sum(okUp),sum(okFin), ...
    100*(1-sum(okUp)/nTotal),100*(1-sum(okFin)/nTotal), ...
    1000*mean(latenciaUp(okUp),'omitnan'), ...
    1000*mean(RTT(okFin),'omitnan'), ...
    1000*std(diff(RTT(okFin)),'omitnan'), ...
    1000*mean(AoI(okUp),'omitnan'), ...
    1000*mean(interGlobal,'omitnan'), ...
    'VariableNames',{'Ntx','NrxServidor','NrxCliente','PerdidaUp_pct', ...
    'PerdidaFinFin_pct','LatenciaUp_ms','RTT_ms','JitterRTT_ms', ...
    'AoI_ms','TactualizacionGlobal_ms'});

disp('METRICAS GLOBALES');
disp(globalMetricas);
disp('METRICAS POR MODULO');
disp(resumen);

%% 9. Exportacion
writetable(registro,'registro_comunicacion_robot.csv');
writetable(resumen,'resumen_comunicacion_por_modulo.csv');
writetable(globalMetricas,'resumen_comunicacion_global.csv');
save('resultados_comunicacion_robot.mat','cfg','registro','resumen', ...
     'globalMetricas');

%% 10. Graficas
figure('Color','w','Name','Comunicacion cliente-servidor');
tiledlayout(2,2,'Padding','compact','TileSpacing','compact');

nexttile;
scatter(tGeneracion(okUp),1000*latenciaUp(okUp),7,modulo(okUp),'filled');
xlabel('Tiempo de generacion [s]'); ylabel('Latencia de subida [ms]');
title('Latencia cliente-servidor'); grid on;
xline(cfg.inicioRafaga_s,'--r'); xline(cfg.finRafaga_s,'--r');

nexttile;
histogram(1000*RTT(okFin),35);
xlabel('Tiempo de ida y vuelta [ms]'); ylabel('Frecuencia');
title('Distribucion del RTT'); grid on;

nexttile;
bar(resumen.Modulo,[resumen.PerdidaUp_pct resumen.PerdidaFinFin_pct]);
xlabel('Modulo'); ylabel('Perdida [%]');
legend('Cliente-servidor','Extremo a extremo','Location','best');
title('Perdida de paquetes'); grid on;

nexttile;
bar(resumen.Modulo,[resumen.LatenciaUp_ms resumen.RTT_ms resumen.AoI_ms]);
xlabel('Modulo'); ylabel('Tiempo [ms]');
legend('Latencia subida','RTT','AoI','Location','best');
title('Metricas temporales'); grid on;

exportgraphics(gcf,'metricas_comunicacion_robot.png','Resolution',300);

fprintf('\nArchivos generados:\n');
fprintf('  registro_comunicacion_robot.csv\n');
fprintf('  resumen_comunicacion_por_modulo.csv\n');
fprintf('  resumen_comunicacion_global.csv\n');
fprintf('  resultados_comunicacion_robot.mat\n');
fprintf('  metricas_comunicacion_robot.png\n');

%% Funcion auxiliar
function y = sat01(x)
    y = min(max(x,0),1);
end
