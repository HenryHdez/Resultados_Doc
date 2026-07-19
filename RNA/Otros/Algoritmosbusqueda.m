function Algoritmosbusqueda

clc; close all;

%% ================= ESTADO GENERAL =================
S.worldSizeX = 1.5;
S.worldSizeY = 2.0;
S.sampleStep = 0.10;

S.maxIter = 1500;
S.k = 1;
S.running = false;

S.numModules = 6;
S.moduleLength = 0.05;
S.moduleWidth  = 0.035;

S.stepSize = 0.015;
S.turnGain = 0.28;

S.theta = pi/5;
S.headPos = [0.15 0.15];

S.waveAmplitude = 0.35;
S.waveFrequency = 1.8;
S.phaseLag = pi/4;

S.trajectory = [];
S.humidityHistory = [];
S.distanceHistory = [];
S.angularErrorHistory = [];
S.phaseHistory = strings(0,1);
S.totalDistance = 0;

S.algorithm = "Metodo hibrido bioinspirado";
S.fieldType = "Irregular";

S.bestHumidity = -inf;
S.bestPos = S.headPos;
S.stayNearMaxCounter = 0;
S.maxStayLimit = 60;

S.noProgressCounter = 0;
S.lastBestHumidity = -inf;
S.escapeCounter = 0;
S.escapeTheta = S.theta;

% Memoria espacial del metodo hibrido
S.topMemory = [];
S.memoryReturnCounter = 0;
S.memoryReturnPeriod = 90;
S.memoryThreshold = 0.70;

% Parametros de visualizacion
S.showTrueField = false;        
S.grayUnknownLevel = 0.72;      % Gris de celdas no descubiertas

%% ================= INTERFAZ =================
fig = uifigure('Name','Robot modular - busqueda bioinspirada unificada', ...
    'Position',[80 80 1180 730]);

ax = uiaxes(fig,'Position',[340 80 800 610]);

uibutton(fig,'push','Text','Iniciar','Position',[30 670 120 30], ...
    'ButtonPushedFcn',@startSim);

uibutton(fig,'push','Text','Detener','Position',[180 670 120 30], ...
    'ButtonPushedFcn',@stopSim);

uibutton(fig,'push','Text','Reiniciar','Position',[30 630 120 30], ...
    'ButtonPushedFcn',@resetSim);

uibutton(fig,'push','Text','Exportar Excel','Position',[180 630 120 30], ...
    'ButtonPushedFcn',@exportExcel);

uibutton(fig,'push', ...
    'Text','Exportar mapa', ...
    'Position',[180 590 120 30], ...
    'ButtonPushedFcn',@exportRouteAndColorMatrix);

uibutton(fig,'push', ...
    'Text','Guardar gráfico', ...
    'Position',[30 590 120 30], ...
    'ButtonPushedFcn',@savePlot);

uilabel(fig,'Text','Velocidad del robot','Position',[30 555 180 25]);
speedSlider = uislider(fig,'Position',[30 540 250 3], ...
    'Limits',[0.003 0.030],'Value',S.stepSize);

uilabel(fig,'Text','Cantidad de modulos','Position',[30 480 180 25]);
moduleSpinner = uispinner(fig,'Position',[30 450 100 30], ...
    'Limits',[3 15],'Value',S.numModules,'Step',1);

uilabel(fig,'Text','Iteraciones maximas','Position',[150 480 180 25]);
iterSpinner = uispinner(fig,'Position',[150 450 100 30], ...
    'Limits',[100 5000],'Value',S.maxIter,'Step',100);

uilabel(fig,'Text','Campo de humedad','Position',[30 420 180 25]);
fieldDrop = uidropdown(fig,'Position',[30 390 250 30], ...
    'Items',{'Irregular','Una fuente','Varias fuentes','Aleatorio'}, ...
    'Value','Irregular');

uilabel(fig,'Text','Metodo de busqueda','Position',[30 360 180 25]);
algDrop = uidropdown(fig,'Position',[30 330 250 30], ...
    'Items',{'Sensor reactivo','Gradiente local','Quimiosintesis', ...
             'Centro de gravedad','GA local','Exploracion por frontera', ...
             'Metodo hibrido bioinspirado'}, ...
    'Value','Metodo hibrido bioinspirado');

showFieldCheck = uicheckbox(fig,'Text','Mostrar campo real completo', ...
    'Position',[30 290 250 25],'Value',false, ...
    'ValueChangedFcn',@toggleTrueField);

uilabel(fig,'Text','Nota: gris = zona no medida / desconocida', ...
    'Position',[30 260 270 25]);

info = uitextarea(fig,'Position',[30 70 280 180], ...
    'Editable','off');

%% ================= CAMPO REAL Y MAPA DESCUBIERTO =================
[S.xGrid,S.yGrid,S.trueHumidity,S.sources] = createHumidityFieldDiscrete( ...
    S.worldSizeX,S.worldSizeY,S.sampleStep,S.fieldType);

S.discoveredMask = false(size(S.trueHumidity));
S.discoveredHumidity = nan(size(S.trueHumidity));
S.visitedCount = zeros(size(S.trueHumidity));

drawEnvironment();

%% ================= CALLBACKS =================
    function startSim(~,~)
        S.running = true;

        while S.running && S.k <= S.maxIter && isvalid(fig)
            S.stepSize = speedSlider.Value;
            S.numModules = moduleSpinner.Value;
            S.maxIter = iterSpinner.Value;
            S.algorithm = string(algDrop.Value);
            simulateStep();
            pause(0.01);
        end
    end

    function stopSim(~,~)
        S.running = false;
    end

    function resetSim(~,~)
        S.running = false;
        S.k = 1;

        S.theta = pi/5;
        S.headPos = [0.15 0.15];

        S.trajectory = [];
        S.humidityHistory = [];
        S.distanceHistory = [];
        S.angularErrorHistory = [];
        S.phaseHistory = strings(0,1);
        S.totalDistance = 0;

        S.bestHumidity = -inf;
        S.bestPos = S.headPos;
        S.stayNearMaxCounter = 0;

        S.noProgressCounter = 0;
        S.lastBestHumidity = -inf;
        S.escapeCounter = 0;
        S.escapeTheta = S.theta;

        S.topMemory = [];
        S.memoryReturnCounter = 0;

        S.numModules = moduleSpinner.Value;
        S.maxIter = iterSpinner.Value;
        S.fieldType = string(fieldDrop.Value);
        S.algorithm = string(algDrop.Value);
        S.showTrueField = showFieldCheck.Value;

        [S.xGrid,S.yGrid,S.trueHumidity,S.sources] = createHumidityFieldDiscrete( ...
            S.worldSizeX,S.worldSizeY,S.sampleStep,S.fieldType);

        S.discoveredMask = false(size(S.trueHumidity));
        S.discoveredHumidity = nan(size(S.trueHumidity));
        S.visitedCount = zeros(size(S.trueHumidity));

        drawEnvironment();
    end

    function toggleTrueField(src,~)
        S.showTrueField = src.Value;
        drawEnvironmentDynamic();
    end

    function exportExcel(~,~)
        if isempty(S.trajectory)
            uialert(fig,'No hay datos para exportar.','Aviso');
            return;
        end

        filename = 'resultados_robot_modular_GUI_unificado.xlsx';
        n = size(S.trajectory,1);

        if isfile(filename)
            try
                oldResumen = readtable(filename,'Sheet','Resumen');
                if any(strcmp(oldResumen.Properties.VariableNames,'RunID')) && ~isempty(oldResumen)
                    runID = max(oldResumen.RunID) + 1;
                else
                    runID = 1;
                end
            catch
                oldResumen = table();
                runID = 1;
            end

            try
                oldTrayectoria = readtable(filename,'Sheet','Trayectoria');
            catch
                oldTrayectoria = table();
            end
        else
            oldResumen = table();
            oldTrayectoria = table();
            runID = 1;
        end

        T = table(repmat(runID,n,1), ...
            (1:n)', ...
            S.trajectory(:,1), ...
            S.trajectory(:,2), ...
            S.humidityHistory(:), ...
            S.distanceHistory(:), ...
            S.angularErrorHistory(:), ...
            S.phaseHistory(:), ...
            'VariableNames',{'RunID','Iteration','X_m','Y_m','HumiditySample', ...
            'AccumulatedDistance_m','AngularError_rad','Phase'});

        resumenNew = table(runID, n, S.totalDistance, ...
            S.humidityHistory(end), max(S.humidityHistory), ...
            mean(S.angularErrorHistory), ...
            string(S.algorithm), string(S.fieldType), S.numModules, ...
            S.sampleStep, S.moduleLength, ...
            100*sum(S.discoveredMask(:))/numel(S.discoveredMask), ...
            sum(S.visitedCount(:)), size(S.topMemory,1), ...
            'VariableNames',{'RunID','Iterations','Distance_m','FinalHumidity', ...
            'BestHumidity','MeanAngularError_rad','Algorithm','FieldType', ...
            'Modules','SampleStep_m','ModuleLength_m','DiscoveredPercent', ...
            'TotalCellVisits','TopMemoryCells'});

        writetable([oldResumen; resumenNew],filename,'Sheet','Resumen');
        writetable([oldTrayectoria; T],filename,'Sheet','Trayectoria');

        uialert(fig,['Ejecucion ',num2str(runID),' agregada al archivo Excel.'], ...
            'Exportacion finalizada');
    end


    function exportRouteAndColorMatrix(~,~)
        % Exporta en un solo archivo Excel:
        % 1) La ruta recorrida por el robot.
        % 2) La matriz de color visible del espacio de busqueda.
        %
        % En la hoja "Matriz_color" cada celda contiene el codigo HEX
        % del color mostrado en la GUI. Las zonas no medidas aparecen
        % como gris, de acuerdo con S.grayUnknownLevel.

        if isempty(S.trajectory)
            uialert(fig,'No hay ruta para exportar. Ejecute primero la simulacion.','Aviso');
            return;
        end

        [file,path] = uiputfile( ...
            {'*.xlsx','Archivo Excel (*.xlsx)'}, ...
            'Exportar ruta y matriz de color', ...
            'ruta_y_matriz_color_robot.xlsx');

        if isequal(file,0)
            return;
        end

        [~,~,ext] = fileparts(file);
        if isempty(ext)
            file = [file '.xlsx'];
        end

        filename = fullfile(path,file);

        try
            n = size(S.trajectory,1);

            rutaTabla = table( ...
                (1:n)', ...
                S.trajectory(:,1), ...
                S.trajectory(:,2), ...
                S.humidityHistory(:), ...
                S.distanceHistory(:), ...
                S.angularErrorHistory(:), ...
                S.phaseHistory(:), ...
                repmat(string(S.algorithm),n,1), ...
                repmat(string(S.fieldType),n,1), ...
                'VariableNames',{'Iteracion','X_m','Y_m','Humedad_muestreada', ...
                'Distancia_acumulada_m','Error_angular_rad','Fase', ...
                'Metodo','Campo'});

            % Matriz RGB actualmente visible en la GUI:
            % - Si S.showTrueField = false, no revela el campo real no medido.
            % - Las celdas desconocidas quedan grises.
            rgb = buildVisibleMap();
            colorHex = rgbToHexMatrix(rgb);

            % Hoja tipo matriz: columnas = coordenadas X, filas = coordenadas Y.
            xVals = S.xGrid(1,:);
            yVals = S.yGrid(:,1);

            matrizColorCell = cell(size(colorHex,1)+1,size(colorHex,2)+1);
            matrizColorCell{1,1} = 'Y_m \\ X_m';

            for c = 1:numel(xVals)
                matrizColorCell{1,c+1} = xVals(c);
            end

            for r = 1:numel(yVals)
                matrizColorCell{r+1,1} = yVals(r);
                for c = 1:numel(xVals)
                    matrizColorCell{r+1,c+1} = colorHex{r,c};
                end
            end

            % Hoja adicional con informacion numerica del mapa.
            % Esta hoja permite reconstruir la visualizacion y analizar
            % celdas medidas/no medidas, humedad y visitas.
            [rows,cols] = size(S.trueHumidity);
            totalCells = rows*cols;

            fila = zeros(totalCells,1);
            columna = zeros(totalCells,1);
            x_m = zeros(totalCells,1);
            y_m = zeros(totalCells,1);
            descubierta = false(totalCells,1);
            humedadDescubierta = nan(totalCells,1);
            humedadReal = zeros(totalCells,1);
            visitas = zeros(totalCells,1);
            colorHEX = strings(totalCells,1);

            q = 1;
            for r = 1:rows
                for c = 1:cols
                    fila(q) = r;
                    columna(q) = c;
                    x_m(q) = S.xGrid(r,c);
                    y_m(q) = S.yGrid(r,c);
                    descubierta(q) = S.discoveredMask(r,c);
                    humedadDescubierta(q) = S.discoveredHumidity(r,c);
                    humedadReal(q) = S.trueHumidity(r,c);
                    visitas(q) = S.visitedCount(r,c);
                    colorHEX(q) = string(colorHex{r,c});
                    q = q + 1;
                end
            end

            mapaTabla = table(fila,columna,x_m,y_m,descubierta, ...
                humedadDescubierta,humedadReal,visitas,colorHEX, ...
                'VariableNames',{'Fila','Columna','X_m','Y_m','Descubierta', ...
                'Humedad_descubierta','Humedad_real','Visitas','Color_HEX'});

            resumenTabla = table( ...
                string(S.algorithm), ...
                string(S.fieldType), ...
                S.maxIter, ...
                n, ...
                S.totalDistance, ...
                S.humidityHistory(end), ...
                max(S.humidityHistory), ...
                mean(S.angularErrorHistory), ...
                100*sum(S.discoveredMask(:))/numel(S.discoveredMask), ...
                S.numModules, ...
                S.sampleStep, ...
                S.moduleLength, ...
                S.showTrueField, ...
                'VariableNames',{'Metodo','Campo','Iteraciones_configuradas', ...
                'Iteraciones_exportadas','Distancia_total_m','Humedad_final', ...
                'Mejor_humedad','Error_angular_promedio_rad', ...
                'Porcentaje_descubierto','Modulos','Paso_muestreo_m', ...
                'Longitud_modulo_m','Campo_real_visible_en_GUI'});

            writetable(rutaTabla,filename,'Sheet','Ruta_robot');
            writecell(matrizColorCell,filename,'Sheet','Matriz_color');
            writetable(mapaTabla,filename,'Sheet','Mapa_celdas');
            writetable(resumenTabla,filename,'Sheet','Resumen');

            uialert(fig,'Ruta y matriz de color exportadas correctamente.','Exportacion finalizada');

        catch ME
            uialert(fig,ME.message,'Error exportando Excel');
        end
    end

    function colorHex = rgbToHexMatrix(rgb)
        rows = size(rgb,1);
        cols = size(rgb,2);
        colorHex = cell(rows,cols);

        rgb255 = round(255*rgb);
        rgb255 = min(max(rgb255,0),255);

        for r = 1:rows
            for c = 1:cols
                R = rgb255(r,c,1);
                G = rgb255(r,c,2);
                B = rgb255(r,c,3);
                colorHex{r,c} = sprintf('#%02X%02X%02X',R,G,B);
            end
        end
    end


%% ================= SIMULACION =================
    function simulateStep()
        [hCurrent,idxCurrent] = readDiscreteSensor(S.xGrid,S.yGrid,S.trueHumidity,S.headPos);
        discoverCell(idxCurrent,hCurrent);
        updateTopMemory(S.headPos,hCurrent);

        S.visitedCount(idxCurrent) = S.visitedCount(idxCurrent) + 1;

        if hCurrent > S.bestHumidity
            S.bestHumidity = hCurrent;
            S.bestPos = S.headPos;
        end

        if S.bestHumidity <= S.lastBestHumidity + 0.002
            S.noProgressCounter = S.noProgressCounter + 1;
        else
            S.noProgressCounter = 0;
        end
        S.lastBestHumidity = S.bestHumidity;

        currentPhase = string(S.algorithm);

        if S.escapeCounter > 0
            desiredTheta = S.escapeTheta + 0.25*randn;
            currentPhase = "Escape";
            S.escapeCounter = S.escapeCounter - 1;
        else
            switch S.algorithm
                case "Sensor reactivo"
                    desiredTheta = sensorReactive();

                case "Gradiente local"
                    desiredTheta = localGradient();

                case "Quimiosintesis"
                    desiredTheta = chemosynthesisSearch();

                case "Centro de gravedad"
                    desiredTheta = centerOfGravitySearch();

                case "GA local"
                    desiredTheta = geneticLocalSearch();

                case "Exploracion por frontera"
                    desiredTheta = frontierExploration();

                case "Metodo hibrido bioinspirado"
                    [desiredTheta,currentPhase] = hybridBioInspiredSearch(hCurrent);

                otherwise
                    desiredTheta = sensorReactive();
            end
        end

        % Escape ante falta de mejora
        if S.noProgressCounter > 85
            S.escapeTheta = S.theta + pi/2 + pi*rand;
            S.escapeCounter = 35;
            S.noProgressCounter = 0;
        end

        % Evita quedarse atrapado en una zona localmente alta
        if hCurrent >= S.bestHumidity - 0.015 && hCurrent > 0.75
            S.stayNearMaxCounter = S.stayNearMaxCounter + 1;
        else
            S.stayNearMaxCounter = 0;
        end

        if S.stayNearMaxCounter > S.maxStayLimit
            S.escapeTheta = S.theta + pi/2 + pi*rand;
            S.escapeCounter = 45;
            S.stayNearMaxCounter = 0;
        end

        angularError = atan2(sin(desiredTheta-S.theta), cos(desiredTheta-S.theta));
        S.theta = S.theta + S.turnGain*angularError;

        newHeadPos = S.headPos + S.stepSize*[cos(S.theta), sin(S.theta)];
        newHeadPos = limitPoint(newHeadPos);

        S.totalDistance = S.totalDistance + norm(newHeadPos-S.headPos);
        S.headPos = newHeadPos;

        S.trajectory(S.k,:) = S.headPos;
        S.humidityHistory(S.k,1) = hCurrent;
        S.distanceHistory(S.k,1) = S.totalDistance;
        S.angularErrorHistory(S.k,1) = abs(angularError);
        S.phaseHistory(S.k,1) = currentPhase;

        drawEnvironmentDynamic();
        drawRobot();
        updateInfo(hCurrent,idxCurrent,currentPhase);

        S.k = S.k + 1;
    end

    function updateInfo(hCurrent,idxCurrent,currentPhase)
        discoveredPercent = 100*sum(S.discoveredMask(:))/numel(S.discoveredMask);

        info.Value = {
            ['Iteracion: ',num2str(S.k),' / ',num2str(S.maxIter)]
            ['Metodo: ',char(S.algorithm)]
            ['Fase: ',char(currentPhase)]
            ['Humedad medida: ',num2str(hCurrent,'%.3f')]
            ['Mejor humedad: ',num2str(S.bestHumidity,'%.3f')]
            ['Mapa descubierto: ',num2str(discoveredPercent,'%.1f'),'%']
            ['Distancia: ',num2str(S.totalDistance,'%.3f'),' m']
            ['Revisitas celda: ',num2str(S.visitedCount(idxCurrent))]
            ['Memoria top: ',num2str(size(S.topMemory,1)),' celdas']
            ['Escape activo: ',num2str(S.escapeCounter)]
            ['Campo: ',char(S.fieldType)]
            };
    end

%% ================= METODO HIBRIDO =================
    function [desiredTheta,phaseName] = hybridBioInspiredSearch(hCurrent)
        discoveredPercent = 100*sum(S.discoveredMask(:))/numel(S.discoveredMask);
        S.memoryReturnCounter = S.memoryReturnCounter + 1;

        if S.memoryReturnCounter >= S.memoryReturnPeriod && ~isempty(S.topMemory)
            phaseName = "Fase 6 - Retorno memoria espacial";
            desiredTheta = memoryReturnSearch();
            S.memoryReturnCounter = 0;
            return;
        end

        if discoveredPercent < 20 %12
            phaseName = "Fase 1 - Exploracion inicial";
            desiredTheta = frontierExploration();

        elseif S.bestHumidity >= 0.75 && hCurrent < 0.70
            phaseName = "Fase 2 - Retorno zona humeda";
            desiredTheta = atan2(S.bestPos(2)-S.headPos(2), S.bestPos(1)-S.headPos(1));

        elseif S.bestHumidity < 0.90 || hCurrent < 0.70 %S.bestHumidity < 0.79
            phaseName = "Fase 3 - Quimiosintesis intensiva";
            desiredTheta = chemosynthesisSearch();

        elseif discoveredPercent < 25 && S.bestHumidity > 0.80 %discoveredPercent < 40
            phaseName = "Fase 4 - Centro gravedad humedo";
            desiredTheta = centerOfGravitySearch(true);

        else
            phaseName = "Fase 5 - Ajuste sensor reactivo";
            desiredTheta = sensorReactive();
        end
    end

    function updateTopMemory(pos,hVal)
        if hVal < S.memoryThreshold
            return;
        end

        if isempty(S.topMemory)
            S.topMemory = [pos hVal];
            return;
        end

        distances = vecnorm(S.topMemory(:,1:2) - pos,2,2);

        if min(distances) < S.sampleStep/2
            [~,idx] = min(distances);
            if hVal > S.topMemory(idx,3)
                S.topMemory(idx,:) = [pos hVal];
            end
        else
            S.topMemory = [S.topMemory; pos hVal];
        end

        [~,order] = sort(S.topMemory(:,3),'descend');
        S.topMemory = S.topMemory(order,:);

        if size(S.topMemory,1) > 10
            S.topMemory = S.topMemory(1:10,:);
        end
    end

    function desiredTheta = memoryReturnSearch()
        if isempty(S.topMemory)
            desiredTheta = frontierExploration();
            return;
        end

        scores = zeros(size(S.topMemory,1),1);

        for i = 1:size(S.topMemory,1)
            target = S.topMemory(i,1:2);
            hVal = S.topMemory(i,3);
            distancePenalty = norm(target - S.headPos);
            scores(i) = hVal - 0.20*distancePenalty;
        end

        [~,idxBest] = max(scores);
        target = S.topMemory(idxBest,1:2);

        if norm(target - S.headPos) < 0.08
            desiredTheta = sensorReactive();
        else
            desiredTheta = atan2(target(2)-S.headPos(2), target(1)-S.headPos(1));
        end

        if isfield(S,'memoryPlot') && isvalid(S.memoryPlot)
            set(S.memoryPlot,'XData',S.topMemory(:,1),'YData',S.topMemory(:,2));
        end
    end

%% ================= ALGORITMOS DE BUSQUEDA =================
    function desiredTheta = sensorReactive()
        sensorDistance = 0.10;
        sensorAngles = [0; pi/6; -pi/6; pi/3; -pi/3; pi/2; -pi/2; pi];

        [hCurrent,~] = readDiscreteSensor(S.xGrid,S.yGrid,S.trueHumidity,S.headPos);

        scores = zeros(length(sensorAngles),1);
        sensorPositions = zeros(length(sensorAngles),2);

        for j = 1:length(sensorAngles)
            alpha = S.theta + sensorAngles(j);
            p = limitPoint(S.headPos + sensorDistance*[cos(alpha), sin(alpha)]);
            sensorPositions(j,:) = p;

            [hVal,idx] = readDiscreteSensor(S.xGrid,S.yGrid,S.trueHumidity,p);
            wasUnknown = ~S.discoveredMask(idx);
            discoverCell(idx,hVal);
            updateTopMemory(p,hVal);

            scores(j) = hVal + ...
                        0.25*double(wasUnknown) - ...
                        0.06*S.visitedCount(idx) - ...
                        0.04*abs(sensorAngles(j));
        end

        prob = softmax(scores,0.12);
        idxBest = stochasticChoice(prob);

        if scores(idxBest) < hCurrent - 0.10
            desiredTheta = frontierExploration();
        else
            desiredTheta = S.theta + sensorAngles(idxBest);
        end

        setSensorPoints(sensorPositions);
    end

    function desiredTheta = localGradient()
        d = S.sampleStep;

        candidateDirs = [
             1  0
            -1  0
             0  1
             0 -1
             1  1
             1 -1
            -1  1
            -1 -1
        ];

        scores = zeros(size(candidateDirs,1),1);
        points = zeros(size(candidateDirs,1),2);

        for j = 1:size(candidateDirs,1)
            dir = candidateDirs(j,:);
            dir = dir / norm(dir);
            p = limitPoint(S.headPos + d*dir);
            points(j,:) = p;

            [hVal,idx] = readDiscreteSensor(S.xGrid,S.yGrid,S.trueHumidity,p);
            wasUnknown = ~S.discoveredMask(idx);
            discoverCell(idx,hVal);
            updateTopMemory(p,hVal);

            angleDir = atan2(dir(2),dir(1));
            turnError = abs(atan2(sin(angleDir-S.theta),cos(angleDir-S.theta)));

            scores(j) = hVal + ...
                        0.30*double(wasUnknown) - ...
                        0.07*S.visitedCount(idx) - ...
                        0.03*turnError;
        end

        prob = softmax(scores,0.10);
        idxBest = stochasticChoice(prob);
        selectedDir = candidateDirs(idxBest,:);
        selectedDir = selectedDir / norm(selectedDir);

        desiredTheta = atan2(selectedDir(2),selectedDir(1));
        setSensorPoints(points);
    end

    function desiredTheta = chemosynthesisSearch()
        [hCurrent,~] = readDiscreteSensor(S.xGrid,S.yGrid,S.trueHumidity,S.headPos);

        sensorDistance = 0.10;
        sensorAngles = [0; pi/6; -pi/6; pi/3; -pi/3; pi/2; -pi/2; pi];

        scores = zeros(length(sensorAngles),1);
        sensorPositions = zeros(length(sensorAngles),2);

        for j = 1:length(sensorAngles)
            alpha = S.theta + sensorAngles(j);
            p = limitPoint(S.headPos + sensorDistance*[cos(alpha), sin(alpha)]);
            sensorPositions(j,:) = p;

            [hVal,idx] = readDiscreteSensor(S.xGrid,S.yGrid,S.trueHumidity,p);
            wasUnknown = ~S.discoveredMask(idx);
            discoverCell(idx,hVal);
            updateTopMemory(p,hVal);

            energyGain = max(hVal - hCurrent, 0);

            scores(j) = 0.55*hVal + ...
                        0.25*energyGain + ...
                        0.30*double(wasUnknown) - ...
                        0.05*S.visitedCount(idx) - ...
                        0.03*abs(sensorAngles(j));
        end

        if hCurrent > 0.75
            temperature = 0.08;
        else
            temperature = 0.18;
        end

        prob = softmax(scores,temperature);
        idxBest = stochasticChoice(prob);
        desiredTheta = S.theta + sensorAngles(idxBest);

        setSensorPoints(sensorPositions);
    end

    function desiredTheta = centerOfGravitySearch(onlyHighHumidity)
        if nargin < 1
            onlyHighHumidity = false;
        end

        mask = S.discoveredMask;

        if sum(mask(:)) < 4
            desiredTheta = frontierExploration();
            return;
        end

        X = S.xGrid(mask);
        Y = S.yGrid(mask);
        H = S.discoveredHumidity(mask);
        H(isnan(H)) = 0;

        if onlyHighHumidity
            highMask = H >= max(0.60,0.80*S.bestHumidity);
            if sum(highMask) >= 3
                X = X(highMask);
                Y = Y(highMask);
                H = H(highMask);
            end
        end

        weights = H.^2 + 0.05;
        cx = sum(X(:).*weights(:))/sum(weights(:));
        cy = sum(Y(:).*weights(:))/sum(weights(:));

        target = [cx cy];

        if norm(target - S.headPos) < 0.08
            desiredTheta = frontierExploration();
        else
            desiredTheta = atan2(target(2)-S.headPos(2), target(1)-S.headPos(1));
        end

        setSensorPoints(target);
    end

    function desiredTheta = geneticLocalSearch()
        populationSize = 18;
        generations = 6;
        mutationStd = pi/8;
        sensorDistance = S.sampleStep;

        population = S.theta + linspace(-pi,pi,populationSize)' + ...
                     0.15*randn(populationSize,1);

        for g = 1:generations
            fitness = zeros(populationSize,1);

            for i = 1:populationSize
                angle = population(i);
                p = limitPoint(S.headPos + sensorDistance*[cos(angle), sin(angle)]);
                [hVal,idx] = readDiscreteSensor(S.xGrid,S.yGrid,S.trueHumidity,p);
                wasUnknown = ~S.discoveredMask(idx);
                turnError = abs(atan2(sin(angle-S.theta),cos(angle-S.theta)));

                fitness(i) = 0.65*hVal + ...
                             0.30*double(wasUnknown) - ...
                             0.06*S.visitedCount(idx) - ...
                             0.03*turnError;
            end

            [~,order] = sort(fitness,'descend');
            elite = population(order(1:4));
            newPopulation = elite;

            while length(newPopulation) < populationSize
                p1 = elite(randi(length(elite)));
                p2 = elite(randi(length(elite)));
                child = atan2(sin((p1+p2)/2),cos((p1+p2)/2));
                child = child + mutationStd*randn;
                newPopulation(end+1,1) = child; %#ok<AGROW>
            end

            population = newPopulation;
        end

        finalFitness = zeros(populationSize,1);
        points = zeros(populationSize,2);

        for i = 1:populationSize
            angle = population(i);
            p = limitPoint(S.headPos + sensorDistance*[cos(angle), sin(angle)]);
            points(i,:) = p;

            [hVal,idx] = readDiscreteSensor(S.xGrid,S.yGrid,S.trueHumidity,p);
            wasUnknown = ~S.discoveredMask(idx);
            discoverCell(idx,hVal);
            updateTopMemory(p,hVal);

            turnError = abs(atan2(sin(angle-S.theta),cos(angle-S.theta)));

            finalFitness(i) = 0.65*hVal + ...
                              0.30*double(wasUnknown) - ...
                              0.06*S.visitedCount(idx) - ...
                              0.03*turnError;
        end

        [~,bestIdx] = max(finalFitness);
        desiredTheta = population(bestIdx);
        setSensorPoints(points);
    end

    function desiredTheta = frontierExploration()
        unknownMask = ~S.discoveredMask;

        if ~any(unknownMask(:))
            desiredTheta = S.theta + 0.8*randn;
            return;
        end

        xUnknown = S.xGrid(unknownMask);
        yUnknown = S.yGrid(unknownMask);

        dist = (xUnknown - S.headPos(1)).^2 + ...
               (yUnknown - S.headPos(2)).^2;

        [~,idxMin] = min(dist);
        target = [xUnknown(idxMin), yUnknown(idxMin)];

        desiredTheta = atan2(target(2)-S.headPos(2), target(1)-S.headPos(1));
        setSensorPoints(target);
    end

%% ================= DESCUBRIMIENTO =================
    function discoverCell(idx,value)
        S.discoveredMask(idx) = true;
        S.discoveredHumidity(idx) = value;
    end

%% ================= DIBUJO =================
    function drawEnvironment()
        cla(ax);
        hold(ax,'off');

        rgb = buildVisibleMap();
        S.mapImage = image(ax,S.xGrid(1,:),S.yGrid(:,1),rgb);
        set(ax,'YDir','normal');
        hold(ax,'on');

        axis(ax,[0 S.worldSizeX 0 S.worldSizeY]);
        axis(ax,'equal');
        xlabel(ax,'x [m]');
        ylabel(ax,'y [m]');
        title(ax,'Campo de busqueda: gris = desconocido; color = celda medida');

        % Malla discreta del espacio de busqueda. Permite ver el campo de busqueda
        % aunque el valor real permanezca oculto por el gris.
        [gx,gy] = meshgrid(0:S.sampleStep:S.worldSizeX, 0:S.sampleStep:S.worldSizeY);
        S.gridPlot = plot(ax,gx,gy,'k.','MarkerSize',5);

        S.trajPlot = plot(ax,nan,nan,'w-','LineWidth',1.8);
        S.moduleLine = plot(ax,nan,nan,'ko-','LineWidth',1.2);
        S.sensorPlot = plot(ax,nan,nan,'ro','MarkerSize',5,'MarkerFaceColor','r');
        S.bestPlot = plot(ax,nan,nan,'mp','MarkerSize',13,'MarkerFaceColor','m');
        S.memoryPlot = plot(ax,nan,nan,'cs','MarkerSize',7,'MarkerFaceColor','c');

        S.robotPatches = gobjects(S.numModules,1);
        for i = 1:S.numModules
            S.robotPatches(i) = patch(ax,nan,nan,[0.75 0.75 0.75], ...
                'EdgeColor','k','LineWidth',1.1);
        end

        drawnow;
    end

    function drawEnvironmentDynamic()
        if ~isfield(S,'mapImage') || ~isvalid(S.mapImage)
            drawEnvironment();
            return;
        end

        rgb = buildVisibleMap();
        set(S.mapImage,'CData',rgb);

        if ~isempty(S.trajectory)
            set(S.trajPlot,'XData',S.trajectory(:,1),'YData',S.trajectory(:,2));
        end

        if isfinite(S.bestHumidity)
            set(S.bestPlot,'XData',S.bestPos(1),'YData',S.bestPos(2));
        end

        if ~isempty(S.topMemory)
            set(S.memoryPlot,'XData',S.topMemory(:,1),'YData',S.topMemory(:,2));
        else
            set(S.memoryPlot,'XData',nan,'YData',nan);
        end
    end

    function rgb = buildVisibleMap()
        cmap = parula(256);

        if S.showTrueField
            rgb = zeros([size(S.trueHumidity),3]);
            for r = 1:size(S.trueHumidity,1)
                for c = 1:size(S.trueHumidity,2)
                    val = S.trueHumidity(r,c);
                    idc = max(1,min(256,round(1 + val*255)));
                    rgb(r,c,:) = cmap(idc,:);
                end
            end
            % Las celdas no descubiertas se cubren con gris semi-informativo:
            % mantiene visible el espacio de busqueda, pero no revela el valor.
            for r = 1:size(S.trueHumidity,1)
                for c = 1:size(S.trueHumidity,2)
                    if ~S.discoveredMask(r,c)
                        rgb(r,c,:) = 0.45*squeeze(rgb(r,c,:))' + ...
                                     0.55*[S.grayUnknownLevel S.grayUnknownLevel S.grayUnknownLevel];
                    end
                end
            end
        else
            rgb = zeros([size(S.trueHumidity),3]) + S.grayUnknownLevel;
            H = S.discoveredHumidity;
            mask = S.discoveredMask;

            for r = 1:size(H,1)
                for c = 1:size(H,2)
                    if mask(r,c)
                        val = H(r,c);
                        idc = max(1,min(256,round(1 + val*255)));
                        rgb(r,c,:) = cmap(idc,:);
                    end
                end
            end
        end
    end

    function drawRobot()
        if ~isfield(S,'robotPatches') || length(S.robotPatches) ~= S.numModules || any(~isvalid(S.robotPatches))
            drawEnvironment();
        end

        t = S.k*0.08;
        centers = zeros(S.numModules,2);
        angles = zeros(S.numModules,1);

        centers(1,:) = S.headPos;
        angles(1) = S.theta;

        for i = 2:S.numModules
            osc = S.waveAmplitude*sin(2*pi*S.waveFrequency*t - (i-1)*S.phaseLag);
            angles(i) = S.theta + osc;
            centers(i,:) = centers(i-1,:) - S.moduleLength*[cos(angles(i)), sin(angles(i))];
        end

        for i = 1:S.numModules
            [xv,yv] = moduleRectangle(centers(i,:),angles(i),S.moduleLength,S.moduleWidth);
            set(S.robotPatches(i),'XData',xv,'YData',yv);
        end

        set(S.moduleLine,'XData',centers(:,1),'YData',centers(:,2));
        drawnow limitrate;
    end

    function setSensorPoints(points)
        if isempty(points)
            set(S.sensorPlot,'XData',nan,'YData',nan);
        else
            set(S.sensorPlot,'XData',points(:,1),'YData',points(:,2));
        end
    end

%% ================= UTILIDADES INTERNAS =================
    function p = limitPoint(p)
        p(1) = min(max(p(1),0),S.worldSizeX);
        p(2) = min(max(p(2),0),S.worldSizeY);
    end

    function prob = softmax(scores,temp)
        scoresNorm = scores - max(scores);
        prob = exp(scoresNorm/temp);
        s = sum(prob);
        if s <= 0 || isnan(s)
            prob = ones(size(scores))/numel(scores);
        else
            prob = prob/s;
        end
    end

    function idx = stochasticChoice(prob)
        r = rand;
        c = cumsum(prob);
        idx = find(r <= c,1,'first');
        if isempty(idx)
            idx = length(prob);
        end
    end

    function savePlot(~,~)
        if ~isvalid(fig) || ~isvalid(ax)
            return;
        end

        [file,path] = uiputfile( ...
            {'*.png','Imagen PNG (*.png)'; ...
             '*.pdf','Documento PDF (*.pdf)'; ...
             '*.fig','Figura MATLAB (*.fig)'}, ...
             'Guardar grafico');

        if isequal(file,0)
            return;
        end

        fullName = fullfile(path,file);
        [~,~,ext] = fileparts(fullName);
        ext = lower(ext);

        try
            switch ext
                case '.png'
                    exportgraphics(ax,fullName,'Resolution',300);

                case '.pdf'
                    exportgraphics(ax,fullName,'ContentType','vector');

                case '.fig'
                    tempFig = figure('Visible','off');
                    tempAx = copyobj(ax,tempFig);
                    set(tempAx,'Units','normalized','Position',[0.13 0.11 0.775 0.815]);
                    savefig(tempFig,fullName);
                    close(tempFig);

                otherwise
                    exportgraphics(ax,fullName,'Resolution',300);
            end

            uialert(fig,'Grafico guardado correctamente.','Exportacion');

        catch ME
            uialert(fig,ME.message,'Error');
        end
    end

end

%% ================================================================
% FUNCIONES AUXILIARES EXTERNAS
% ================================================================
function [xGrid,yGrid,H,sources] = createHumidityFieldDiscrete(worldSizeX,worldSizeY,sampleStep,fieldType)

[xGrid,yGrid] = meshgrid(0:sampleStep:worldSizeX, 0:sampleStep:worldSizeY);
H = zeros(size(xGrid));
sources = [];

switch string(fieldType)
    case "Una fuente"
        sources = [1.25 1.70 1.00 0.25];

    case "Varias fuentes"
        sources = [
            1.25 1.70 1.00 0.25
            0.45 1.30 0.65 0.20
            1.05 0.55 0.55 0.18
        ];

    case "Irregular"
        sources = [
            1.25 1.70 1.00 0.25
            0.45 1.30 0.65 0.20
            1.05 0.55 0.55 0.18
            0.25 0.35 0.35 0.15
        ];

    case "Aleatorio"
        n = 5;
        sources = [
            worldSizeX*rand(n,1), ...
            worldSizeY*rand(n,1), ...
            0.3 + 0.7*rand(n,1), ...
            0.12 + 0.18*rand(n,1)
        ];
end

for s = 1:size(sources,1)
    cx = sources(s,1);
    cy = sources(s,2);
    amp = sources(s,3);
    sig = sources(s,4);

    H = H + amp*exp(-((xGrid-cx).^2 + (yGrid-cy).^2)/(2*sig^2));
end

if fieldType == "Irregular" || fieldType == "Aleatorio"
    H = H + 0.08*randn(size(H));
end

H = H - min(H(:));
if max(H(:)) > 0
    H = H / max(H(:));
end

end

function [h,idx] = readDiscreteSensor(xGrid,yGrid,H,pos)
dist = (xGrid - pos(1)).^2 + (yGrid - pos(2)).^2;
[~,idx] = min(dist(:));
h = H(idx);
end

function [xv,yv] = moduleRectangle(center,angle,L,W)
local = [
     L/2  W/2
     L/2 -W/2
    -L/2 -W/2
    -L/2  W/2
]';

R = [
    cos(angle) -sin(angle)
    sin(angle)  cos(angle)
];

pts = R*local + center';
xv = pts(1,:);
yv = pts(2,:);
end
