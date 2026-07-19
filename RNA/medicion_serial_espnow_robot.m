%% MEDICION_SERIAL_ESPNOW_ROBOT
% Medicion de la arquitectura real del robot:
%
% MATLAB/PC <--- USB-SERIAL ---> ESP32 MAESTRO <--- ESP-NOW ---> MODULOS
%
% MATLAB envia al maestro:
%   PING,<id_modulo>,<secuencia>\n
%
% El maestro reenvia la solicitud por ESP-NOW. Cuando recibe la respuesta
% del modulo, devuelve por serial:
%   PONG,<id>,<secuencia>,<t_envio_esp_us>,<t_recepcion_esp_us>,<rssi>\n
%
% t_envio_esp_us y t_recepcion_esp_us se obtienen con esp_timer_get_time()
% en el ESP32 maestro. Por tanto:
%   RTT total       = tiempo desde que MATLAB envia hasta que recibe PONG.
%   RTT ESP-NOW     = (t_recepcion_esp_us-t_envio_esp_us)/1000.
%   Sobrecarga local= RTT total-RTT ESP-NOW.
%
% Si el firmware no reporta tiempos o RSSI, puede responder solamente:
%   PONG,<id>,<secuencia>
% y el script conservara la medicion del RTT total.

clear; clc; close all;

%% 1. CONFIGURACION
cfg.puertoSerial     = "COM7";    % Cambiar por el puerto del ESP32 maestro
cfg.baudios          = 115200;
cfg.timeoutSerial_s  = 0.25;
cfg.periodo_s        = 0.050;     % Periodo de consulta: 50 ms
cfg.duracion_s       = 60;
cfg.idsModulos       = [1 2 3 4 5 6];
cfg.exportar         = true;

% Para identificar el puerto en Windows puede ejecutar: serialportlist
disp('Puertos seriales disponibles:');
disp(serialportlist("available"));

%% 2. CONEXION CON EL ESP32 MAESTRO
s = serialport(cfg.puertoSerial,cfg.baudios,"Timeout",cfg.timeoutSerial_s);
configureTerminator(s,"LF");
flush(s);
pause(1);                         % Tiempo para estabilizar el puerto
flush(s);

fprintf('Conectado al maestro por %s a %d baudios.\n', ...
    cfg.puertoSerial,cfg.baudios);

%% 3. ESTRUCTURAS DE REGISTRO
nModulos = numel(cfg.idsModulos);
nCiclos = ceil(cfg.duracion_s/cfg.periodo_s);
nMax = nModulos*nCiclos;

idModulo       = nan(nMax,1);
secuencia      = nan(nMax,1);
tEnvioPC       = nan(nMax,1);
tRecepcionPC   = nan(nMax,1);
RTTtotal_ms    = nan(nMax,1);
tEnvioESP_us   = nan(nMax,1);
tRecepcionESP_us = nan(nMax,1);
RTTespnow_ms   = nan(nMax,1);
sobrecarga_ms  = nan(nMax,1);
RSSI_dBm       = nan(nMax,1);
respondio      = false(nMax,1);
fueraTimeout   = false(nMax,1);
respuestaRaw   = strings(nMax,1);

pendientes = containers.Map('KeyType','char','ValueType','double');
seq = zeros(nModulos,1);
fila = 0;
t0 = tic;
proximoEnvio = 0;

%% 4. MEDICION
try
    while toc(t0) < cfg.duracion_s
        ahora = toc(t0);

        % Consulta secuencial a los modulos en cada periodo.
        if ahora >= proximoEnvio
            for m = 1:nModulos
                fila = fila+1;
                seq(m) = seq(m)+1;
                id = cfg.idsModulos(m);
                clave = sprintf('%d_%d',id,seq(m));

                tEnvioPC(fila) = toc(t0);
                idModulo(fila) = id;
                secuencia(fila) = seq(m);
                pendientes(clave) = fila;

                writeline(s,sprintf('PING,%d,%d',id,seq(m)));
            end
            proximoEnvio = proximoEnvio+cfg.periodo_s;
        end

        % Procesa todas las lineas entregadas por el maestro.
        while s.NumBytesAvailable > 0
            linea = strtrim(readline(s));
            tRx = toc(t0);
            campos = split(linea,',');

            if numel(campos) >= 3 && strcmpi(strtrim(campos(1)),"PONG")
                idRx = str2double(campos(2));
                seqRx = str2double(campos(3));
                clave = sprintf('%d_%d',idRx,seqRx);

                if isKey(pendientes,clave)
                    f = pendientes(clave);
                    tRecepcionPC(f) = tRx;
                    RTTtotal_ms(f) = 1000*(tRx-tEnvioPC(f));
                    respondio(f) = true;
                    fueraTimeout(f) = RTTtotal_ms(f) > 1000*cfg.timeoutSerial_s;
                    respuestaRaw(f) = linea;

                    % Campos opcionales reportados por el ESP32 maestro.
                    if numel(campos) >= 5
                        tEnvioESP_us(f) = str2double(campos(4));
                        tRecepcionESP_us(f) = str2double(campos(5));
                        RTTespnow_ms(f) = ...
                            (tRecepcionESP_us(f)-tEnvioESP_us(f))/1000;
                        sobrecarga_ms(f) = RTTtotal_ms(f)-RTTespnow_ms(f);
                    end
                    if numel(campos) >= 6
                        RSSI_dBm(f) = str2double(campos(6));
                    end
                    remove(pendientes,clave);
                end
            end
        end
        pause(0.001);
    end
catch ME
    warning('La medicion se interrumpio: %s',ME.message);
end

% Recibe las respuestas que todavia se encuentren en transito.
tEspera = tic;
while toc(tEspera) < cfg.timeoutSerial_s
    if s.NumBytesAvailable > 0
        linea = strtrim(readline(s));
        tRx = toc(t0);
        campos = split(linea,',');
        if numel(campos) >= 3 && strcmpi(strtrim(campos(1)),"PONG")
            idRx = str2double(campos(2));
            seqRx = str2double(campos(3));
            clave = sprintf('%d_%d',idRx,seqRx);
            if isKey(pendientes,clave)
                f = pendientes(clave);
                tRecepcionPC(f) = tRx;
                RTTtotal_ms(f) = 1000*(tRx-tEnvioPC(f));
                respondio(f) = true;
                fueraTimeout(f) = RTTtotal_ms(f) > 1000*cfg.timeoutSerial_s;
                respuestaRaw(f) = linea;
                if numel(campos) >= 5
                    tEnvioESP_us(f) = str2double(campos(4));
                    tRecepcionESP_us(f) = str2double(campos(5));
                    RTTespnow_ms(f) = ...
                        (tRecepcionESP_us(f)-tEnvioESP_us(f))/1000;
                    sobrecarga_ms(f) = RTTtotal_ms(f)-RTTespnow_ms(f);
                end
                if numel(campos) >= 6
                    RSSI_dBm(f) = str2double(campos(6));
                end
                remove(pendientes,clave);
            end
        end
    end
    pause(0.001);
end

clear s;

%% 5. REGISTRO DE PAQUETES
u = 1:fila;
estado = strings(fila,1);
estado(~respondio(u)) = "Sin_respuesta";
estado(respondio(u) & ~fueraTimeout(u)) = "Recibido";
estado(respondio(u) & fueraTimeout(u)) = "Fuera_timeout";

registro = table(idModulo(u),secuencia(u),tEnvioPC(u),tRecepcionPC(u), ...
    RTTtotal_ms(u),tEnvioESP_us(u),tRecepcionESP_us(u),RTTespnow_ms(u), ...
    sobrecarga_ms(u),RSSI_dBm(u),respondio(u),fueraTimeout(u),estado, ...
    respuestaRaw(u),'VariableNames',{'Modulo','Secuencia','T_envio_PC_s', ...
    'T_recepcion_PC_s','RTT_total_ms','T_envio_ESPNOW_us', ...
    'T_recepcion_ESPNOW_us','RTT_ESPNOW_ms','Serial_procesamiento_ms', ...
    'RSSI_dBm','Respondio','Fuera_timeout','Estado','Respuesta'});

%% 6. METRICAS POR MODULO
resumen = table('Size',[nModulos 13], ...
    'VariableTypes',repmat("double",1,13), ...
    'VariableNames',{'Modulo','N_tx','N_rx','Perdida_pct','RTT_total_ms', ...
    'RTT_total_p95_ms','Jitter_total_ms','RTT_ESPNOW_ms', ...
    'Jitter_ESPNOW_ms','Serial_procesamiento_ms','RSSI_medio_dBm', ...
    'Timeout_pct','Tasa_efectiva_Hz'});

for m = 1:nModulos
    id = cfg.idsModulos(m);
    im = registro.Modulo == id;
    ok = im & registro.Respondio;
    rtt = registro.RTT_total_ms(ok);
    rttEsp = registro.RTT_ESPNOW_ms(ok);
    tiemposRx = sort(registro.T_recepcion_PC_s(ok));

    resumen.Modulo(m) = id;
    resumen.N_tx(m) = sum(im);
    resumen.N_rx(m) = sum(ok);
    resumen.Perdida_pct(m) = 100*(1-sum(ok)/sum(im));
    resumen.RTT_total_ms(m) = mean(rtt,'omitnan');
    resumen.RTT_total_p95_ms(m) = percentilSeguro(rtt,95);
    resumen.Jitter_total_ms(m) = std(diff(rtt),'omitnan');
    resumen.RTT_ESPNOW_ms(m) = mean(rttEsp,'omitnan');
    resumen.Jitter_ESPNOW_ms(m) = std(diff(rttEsp),'omitnan');
    resumen.Serial_procesamiento_ms(m) = ...
        mean(registro.Serial_procesamiento_ms(ok),'omitnan');
    resumen.RSSI_medio_dBm(m) = mean(registro.RSSI_dBm(ok),'omitnan');
    resumen.Timeout_pct(m) = 100*sum(registro.Fuera_timeout(im))/sum(im);
    resumen.Tasa_efectiva_Hz(m) = 1/mean(diff(tiemposRx),'omitnan');
end

disp('METRICAS DE LA ARQUITECTURA SERIAL + ESP-NOW');
disp(resumen);

%% 7. EXPORTACION
if cfg.exportar
    marca = datestr(now,'yyyymmdd_HHMMSS');
    archivoRegistro = ['registro_serial_espnow_' marca '.csv'];
    archivoResumen = ['resumen_serial_espnow_' marca '.csv'];
    archivoMAT = ['medicion_serial_espnow_' marca '.mat'];
    archivoFigura = ['metricas_serial_espnow_' marca '.png'];

    writetable(registro,archivoRegistro);
    writetable(resumen,archivoResumen);
    save(archivoMAT,'cfg','registro','resumen');

    figure('Color','w','Name','Serial y ESP-NOW');
    tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
    nexttile; hold on;
    for m = 1:nModulos
        im = registro.Modulo == cfg.idsModulos(m) & registro.Respondio;
        plot(registro.T_recepcion_PC_s(im),registro.RTT_total_ms(im),'.', ...
            'DisplayName',sprintf('Modulo %d',cfg.idsModulos(m)));
    end
    yline(1000*cfg.timeoutSerial_s,'--r','Timeout');
    xlabel('Tiempo [s]'); ylabel('RTT total [ms]');
    title('Retardo extremo a extremo'); grid on; legend('Location','best');

    nexttile;
    bar(resumen.Modulo,[resumen.RTT_total_ms resumen.RTT_ESPNOW_ms]);
    xlabel('Modulo'); ylabel('Tiempo [ms]');
    legend('RTT total','RTT ESP-NOW','Location','best');
    title('Contribucion del enlace ESP-NOW'); grid on;
    exportgraphics(gcf,archivoFigura,'Resolution',300);

    fprintf('\nArchivos generados:\n%s\n%s\n%s\n%s\n', ...
        archivoRegistro,archivoResumen,archivoMAT,archivoFigura);
end

%% FUNCION AUXILIAR
function p = percentilSeguro(x,q)
    x = sort(x(~isnan(x)));
    if isempty(x)
        p = NaN;
        return;
    end
    pos = 1+(numel(x)-1)*q/100;
    i = floor(pos); j = ceil(pos);
    if i == j
        p = x(i);
    else
        p = x(i)+(pos-i)*(x(j)-x(i));
    end
end
