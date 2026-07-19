function Algprueba

clc; close all;

%% ================= ESTADO GENERAL =================
S.worldSizeX = 1.5;
S.worldSizeY = 2.0;
S.sampleStep = 0.10;

S.maxIter = 1500;
S.k = 1;
S.running = false;

S.numModules = 6;
S.robotShape = "Cadena";
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

% Datos para aprendizaje por imitacion. Las entradas son exclusivamente
% variables disponibles en el robot fisico; X, Y y mapa real no se usan.
S.rnaFeatures = zeros(0,11);
S.rnaLabels = zeros(0,1);
S.rnaRunIDs = zeros(0,1);
S.rnaRunID = 1;
S.previousAction = 1;       % 1 avanzar, 2 izquierda, 3 derecha, 4 evasion
S.humidityWindow = nan(5,1);
S.virtualSignal = 0;
S.virtualDecay = 0.15;

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

uibutton(fig,'push','Text','Entrenar y exportar RNA', ...
    'Position',[30 550 270 30], ...
    'ButtonPushedFcn',@trainActionRNA);

uibutton(fig,'push','Text','Exportar datos RNA en bruto', ...
    'Position',[30 295 270 30], ...
    'ButtonPushedFcn',@exportRawRNAData);

uibutton(fig,'push', ...
    'Text','Exportar mapa', ...
    'Position',[180 590 120 30], ...
    'ButtonPushedFcn',@exportRouteAndColorMatrix);

uibutton(fig,'push', ...
    'Text','Guardar gráfico', ...
    'Position',[30 590 120 30], ...
    'ButtonPushedFcn',@savePlot);

uilabel(fig,'Text','Velocidad del robot','Position',[30 525 180 25]);
speedSlider = uislider(fig,'Position',[30 510 250 3], ...
    'Limits',[0.003 0.030],'Value',S.stepSize);

uilabel(fig,'Text','Cantidad de modulos','Position',[30 480 180 25]);
moduleSpinner = uispinner(fig,'Position',[30 450 100 30], ...
    'Limits',[3 31],'Value',S.numModules,'Step',1, ...
    'ValueChangedFcn',@updateRobotShapeFromGUI);

uilabel(fig,'Text','Morfologia','Position',[30 525 180 25]);
shapeDrop = uidropdown(fig,'Position',[150 520 130 30], ...
    'Items',{'Cadena','T','Cruz +','L'}, ...
    'Value','Cadena', ...
    'ValueChangedFcn',@updateRobotShapeFromGUI);

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
    'Position',[30 265 250 25],'Value',false, ...
    'ValueChangedFcn',@toggleTrueField);

uilabel(fig,'Text','Nota: gris = zona no medida / desconocida', ...
    'Position',[30 235 270 25]);

info = uitextarea(fig,'Position',[30 45 280 180], ...
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
            S.robotShape = string(shapeDrop.Value);
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
        % Los patrones anteriores se conservan para acumular varias
        % ejecuciones y diferentes campos antes del entrenamiento.
        S.rnaRunID = S.rnaRunID + 1;
        S.previousAction = 1;
        S.humidityWindow = nan(5,1);
        S.virtualSignal = 0;

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
        S.robotShape = string(shapeDrop.Value);
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

    function updateRobotShapeFromGUI(~,~)
        S.numModules = moduleSpinner.Value;
        S.robotShape = string(shapeDrop.Value);

        if isfield(S,'robotPatches')
            try
                delete(S.robotPatches(isvalid(S.robotPatches)));
            catch
            end
        end

        S.robotPatches = gobjects(S.numModules,1);
        for ii = 1:S.numModules
            S.robotPatches(ii) = patch(ax,nan,nan,[0.75 0.75 0.75], ...
                'EdgeColor','k','LineWidth',1.1);
        end

        drawEnvironmentDynamic();
        drawRobot();
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
            string(S.algorithm), string(S.fieldType), string(S.robotShape), S.numModules, ...
            S.sampleStep, S.moduleLength, ...
            100*sum(S.discoveredMask(:))/numel(S.discoveredMask), ...
            sum(S.visitedCount(:)), size(S.topMemory,1), ...
            'VariableNames',{'RunID','Iterations','Distance_m','FinalHumidity', ...
            'BestHumidity','MeanAngularError_rad','Algorithm','FieldType','Morphology', ...
            'Modules','SampleStep_m','ModuleLength_m','DiscoveredPercent', ...
            'TotalCellVisits','TopMemoryCells'});

        writetable([oldResumen; resumenNew],filename,'Sheet','Resumen');
        writetable([oldTrayectoria; T],filename,'Sheet','Trayectoria');

        uialert(fig,['Ejecucion ',num2str(runID),' agregada al archivo Excel.'], ...
            'Exportacion finalizada');
    end

    function trainActionRNA(~,~)
        % Entrena una RNA clasificadora por imitacion del controlador experto.
        % Entradas: humedad, historial, proximidad, senal virtual y accion previa.
        % Salidas: 1 avance, 2 izquierda, 3 derecha, 4 evasion.
        if size(S.rnaFeatures,1) < 100
            uialert(fig,['Ejecute primero la simulacion. Se requieren al menos ' ...
                '100 patrones para entrenar la RNA.'],'Datos insuficientes');
            return;
        end
        if exist('patternnet','file') ~= 2
            uialert(fig,'Se requiere Deep Learning Toolbox (patternnet).', ...
                'Herramienta no disponible');
            return;
        end

        [file,path] = uiputfile({'*.xlsx','Archivo Excel (*.xlsx)'}, ...
            'Exportar datos y RNA','RNA_navegacion_sensorial.xlsx');
        if isequal(file,0), return; end
        [~,~,ext] = fileparts(file);
        if isempty(ext), file = [file '.xlsx']; end
        filename = fullfile(path,file);

        try
            rng(42,'twister');
            featureNames = {'Humedad_k','Humedad_k_1','Humedad_k_2', ...
                'Delta_humedad','Promedio_humedad','Pendiente_humedad', ...
                'Proximidad_frente','Proximidad_izquierda','Proximidad_derecha', ...
                'Senal_virtual_C','Accion_anterior'};
            X = S.rnaFeatures';
            labels = S.rnaLabels(:)';
            nClasses = 4;
            T = full(ind2vec(labels,nClasses));

            net = patternnet([32 16 8],'trainscg','crossentropy');
            net.name = 'RNA_navegacion_sensorial';
            net.inputs{1}.processFcns = {'removeconstantrows','mapminmax'};
            net.layers{1}.transferFcn = 'tansig';
            net.layers{2}.transferFcn = 'tansig';
            net.layers{3}.transferFcn = 'tansig';
            net.layers{4}.transferFcn = 'softmax';
            uniqueRuns = unique(S.rnaRunIDs,'stable');
            if numel(uniqueRuns) >= 3
                shuffledRuns = uniqueRuns(randperm(numel(uniqueRuns)));
                nTrainRuns = max(1,floor(0.70*numel(shuffledRuns)));
                nValRuns = max(1,floor(0.15*numel(shuffledRuns)));
                if nTrainRuns+nValRuns >= numel(shuffledRuns)
                    nTrainRuns = numel(shuffledRuns)-2;
                    nValRuns = 1;
                end
                trainRuns = shuffledRuns(1:nTrainRuns);
                valRuns = shuffledRuns(nTrainRuns+1:nTrainRuns+nValRuns);
                testRuns = shuffledRuns(nTrainRuns+nValRuns+1:end);
                net.divideFcn = 'divideind';
                net.divideParam.trainInd = find(ismember(S.rnaRunIDs,trainRuns))';
                net.divideParam.valInd = find(ismember(S.rnaRunIDs,valRuns))';
                net.divideParam.testInd = find(ismember(S.rnaRunIDs,testRuns))';
            else
                % Modo preliminar cuando solo se ha ejecutado uno o dos mapas.
                net.divideFcn = 'dividerand';
                net.divideParam.trainRatio = 0.70;
                net.divideParam.valRatio = 0.15;
                net.divideParam.testRatio = 0.15;
            end
            net.trainParam.epochs = 500;
            net.trainParam.max_fail = 20;
            net.trainParam.showWindow = true;

            [net,tr] = train(net,X,T);
            scores = net(X);
            [confidence,predicted] = max(scores,[],1);

            partition = strings(numel(labels),1);
            partition(tr.trainInd) = "Entrenamiento";
            partition(tr.valInd) = "Validacion";
            partition(tr.testInd) = "Prueba";

            classNames = ["Avanzar";"Izquierda";"Derecha";"Evasion"];
            setNames = ["Entrenamiento";"Validacion";"Prueba";"Total"];
            idxSets = {tr.trainInd,tr.valInd,tr.testInd,1:numel(labels)};
            accuracy = zeros(4,1); balancedAccuracy = zeros(4,1);
            sampleCount = zeros(4,1);
            for q = 1:4
                idx = idxSets{q}; sampleCount(q) = numel(idx);
                accuracy(q) = mean(predicted(idx)==labels(idx));
                recallClass = nan(nClasses,1);
                for c = 1:nClasses
                    idc = idx(labels(idx)==c);
                    if ~isempty(idc)
                        recallClass(c) = mean(predicted(idc)==c);
                    end
                end
                balancedAccuracy(q) = mean(recallClass,'omitnan');
            end
            metricsTable = table(setNames,sampleCount,accuracy,balancedAccuracy, ...
                'VariableNames',{'Conjunto','Muestras','Exactitud','Exactitud_balanceada'});

            dataTable = array2table(S.rnaFeatures,'VariableNames',featureNames);
            dataTable.SampleID = (1:height(dataTable))';
            dataTable.RunID = S.rnaRunIDs;
            dataTable.Accion_experta = labels';
            dataTable.Nombre_accion_experta = classNames(labels);
            dataTable.Accion_predicha = predicted';
            dataTable.Nombre_accion_predicha = classNames(predicted);
            dataTable.Confianza = confidence';
            dataTable.Particion = partition;
            dataTable = movevars(dataTable,{'SampleID','RunID'},'Before',1);

            confusionTotal = confusionmat(labels,predicted,'Order',1:nClasses);
            confusionTable = array2table(confusionTotal, ...
                'VariableNames',cellstr("Pred_"+classNames));
            confusionTable = addvars(confusionTable,classNames, ...
                'Before',1,'NewVariableNames','Clase_real');

            classCount = accumarray(labels',1,[nClasses 1]);
            classTable = table((1:nClasses)',classNames,classCount, ...
                classCount/sum(classCount), ...
                'VariableNames',{'Clase','Accion','Muestras','Proporcion'});

            configTable = table( ...
                ["Tipo";"Entradas";"Salida";"Arquitectura";"Entrenamiento"; ...
                 "Perdida";"Division";"Semilla";"Muestras"], ...
                ["Clasificacion por imitacion";strjoin(string(featureNames),', '); ...
                 "4 acciones";"11-32-16-8-4";"trainscg";"crossentropy"; ...
                 string(net.divideFcn);"42";string(numel(labels))], ...
                'VariableNames',{'Parametro','Valor'});

            % Pesos y sesgos aprendidos.
            weightSheets = {'Pesos_capa_1','Pesos_capa_2','Pesos_capa_3','Pesos_salida'};
            weights = {net.IW{1,1},net.LW{2,1},net.LW{3,2},net.LW{4,3}};
            if isfile(filename), delete(filename); end
            writetable(configTable,filename,'Sheet','Configuracion');
            writetable(metricsTable,filename,'Sheet','Metricas');
            writetable(classTable,filename,'Sheet','Balance_clases');
            writetable(confusionTable,filename,'Sheet','Matriz_confusion');
            writetable(dataTable,filename,'Sheet','Datos_RNA');
            for q = 1:numel(weights)
                writematrix(weights{q},filename,'Sheet',weightSheets{q});
            end
            biasTable = table( ...
                [repmat("Capa_1",numel(net.b{1}),1);repmat("Capa_2",numel(net.b{2}),1); ...
                 repmat("Capa_3",numel(net.b{3}),1);repmat("Salida",numel(net.b{4}),1)], ...
                [(1:numel(net.b{1}))';(1:numel(net.b{2}))'; ...
                 (1:numel(net.b{3}))';(1:numel(net.b{4}))'], ...
                [net.b{1};net.b{2};net.b{3};net.b{4}], ...
                'VariableNames',{'Capa','Neurona','Sesgo'});
            writetable(biasTable,filename,'Sheet','Sesgos');

            [folder,base,~] = fileparts(filename);
            save(fullfile(folder,[base '.mat']),'net','tr','featureNames','classNames');
            uialert(fig,sprintf(['RNA entrenada. Exactitud de prueba: %.2f %%\n' ...
                'Exactitud balanceada: %.2f %%'],100*accuracy(3), ...
                100*balancedAccuracy(3)),'Entrenamiento finalizado');
        catch ME
            uialert(fig,ME.message,'Error entrenando la RNA');
        end
    end

    function exportRawRNAData(~,~)
        % Exporta los patrones acumulados sin normalizar, dividir ni entrenar.
        if isempty(S.rnaFeatures)
            uialert(fig,['No hay patrones disponibles. Ejecute al menos una ' ...
                'simulacion antes de exportar.'],'Datos no disponibles');
            return;
        end

        [file,path] = uiputfile({'*.xlsx','Archivo Excel (*.xlsx)'}, ...
            'Exportar datos de entrenamiento en bruto', ...
            'datos_RNA_navegacion_en_bruto.xlsx');
        if isequal(file,0), return; end
        [~,~,ext] = fileparts(file);
        if isempty(ext), file = [file '.xlsx']; end
        filename = fullfile(path,file);

        try
            featureNames = {'Humedad_k','Humedad_k_1','Humedad_k_2', ...
                'Delta_humedad','Promedio_humedad','Pendiente_humedad', ...
                'Proximidad_frente','Proximidad_izquierda','Proximidad_derecha', ...
                'Senal_virtual_C','Accion_anterior'};
            actionNames = ["Avanzar";"Izquierda";"Derecha";"Evasion"];
            n = size(S.rnaFeatures,1);

            rawTable = array2table(S.rnaFeatures,'VariableNames',featureNames);
            rawTable.SampleID = (1:n)';
            rawTable.RunID = S.rnaRunIDs;
            rawTable.Iteracion_run = zeros(n,1);
            for runValue = unique(S.rnaRunIDs)'
                idxRun = find(S.rnaRunIDs==runValue);
                rawTable.Iteracion_run(idxRun) = (1:numel(idxRun))';
            end
            rawTable.Accion_experta = S.rnaLabels;
            rawTable.Nombre_accion = actionNames(S.rnaLabels);
            rawTable = movevars(rawTable, ...
                {'SampleID','RunID','Iteracion_run'},'Before',1);

            uniqueRuns = unique(S.rnaRunIDs,'stable');
            runSamples = zeros(numel(uniqueRuns),1);
            meanHumidity = zeros(numel(uniqueRuns),1);
            maxHumidity = zeros(numel(uniqueRuns),1);
            for q = 1:numel(uniqueRuns)
                idx = S.rnaRunIDs==uniqueRuns(q);
                runSamples(q) = sum(idx);
                meanHumidity(q) = mean(S.rnaFeatures(idx,1));
                maxHumidity(q) = max(S.rnaFeatures(idx,1));
            end
            runTable = table(uniqueRuns,runSamples,meanHumidity,maxHumidity, ...
                'VariableNames',{'RunID','Muestras','Humedad_media','Humedad_maxima'});

            classCount = accumarray(S.rnaLabels,1,[4 1]);
            classTable = table((1:4)',actionNames,classCount, ...
                classCount/sum(classCount), ...
                'VariableNames',{'Clase','Accion','Muestras','Proporcion'});

            dictionaryTable = table( ...
                string([featureNames,{'Accion_experta'}])', ...
                [repmat("Entrada de la RNA",numel(featureNames),1); ...
                 "Etiqueta generada por el controlador experto"], ...
                [repmat("Valor numerico sin transformacion",numel(featureNames),1); ...
                 "1=Avanzar, 2=Izquierda, 3=Derecha, 4=Evasion"], ...
                'VariableNames',{'Variable','Funcion','Codificacion'});

            if isfile(filename), delete(filename); end
            writetable(rawTable,filename,'Sheet','Datos_brutos');
            writetable(runTable,filename,'Sheet','Resumen_ejecuciones');
            writetable(classTable,filename,'Sheet','Balance_clases');
            writetable(dictionaryTable,filename,'Sheet','Diccionario_variables');

            uialert(fig,sprintf(['Se exportaron %d patrones pertenecientes ' ...
                'a %d ejecuciones.'],n,numel(uniqueRuns)), ...
                'Exportacion finalizada');
        catch ME
            uialert(fig,ME.message,'Error exportando datos en bruto');
        end
    end

    function trainAndExportRNA(~,~)
        % Entrena fuera de linea una RNA de regresion espacial:
        %       [x,y] -> humedad normalizada
        % y exporta datos, metricas, predicciones, pesos y sesgos a Excel.
        % Requiere Deep Learning Toolbox (fitnet/train).

        if exist('fitnet','file') ~= 2
            uialert(fig,['No se encontro Deep Learning Toolbox. ' ...
                'La funcion fitnet es necesaria para entrenar la RNA.'], ...
                'Herramienta no disponible');
            return;
        end

        [file,path] = uiputfile({'*.xlsx','Archivo Excel (*.xlsx)'}, ...
            'Exportar parametros de la RNA','parametros_RNA_humedad.xlsx');
        if isequal(file,0)
            return;
        end
        [~,~,ext] = fileparts(file);
        if isempty(ext)
            file = [file '.xlsx'];
        end
        filename = fullfile(path,file);

        try
            rng(42,'twister');

            % Cada celda del campo simulado constituye un patron etiquetado.
            x = S.xGrid(:);
            y = S.yGrid(:);
            humidity = S.trueHumidity(:);
            X = [x'; y'];
            Y = humidity';
            nSamples = numel(humidity);

            % Arquitectura de regresion con dos capas ocultas.
            hiddenLayers = [16 8];
            net = fitnet(hiddenLayers,'trainlm');
            net.name = 'RNA_predictiva_humedad_espacial';
            net.inputs{1}.processFcns = {'removeconstantrows','mapminmax'};
            net.outputs{3}.processFcns = {'removeconstantrows','mapminmax'};
            net.layers{1}.transferFcn = 'tansig';
            net.layers{2}.transferFcn = 'tansig';
            net.layers{3}.transferFcn = 'purelin';
            net.performFcn = 'mse';
            net.divideFcn = 'dividerand';
            net.divideMode = 'sample';
            net.divideParam.trainRatio = 0.70;
            net.divideParam.valRatio = 0.15;
            net.divideParam.testRatio = 0.15;
            net.trainParam.epochs = 1000;
            net.trainParam.max_fail = 20;
            net.trainParam.min_grad = 1e-7;
            net.trainParam.showWindow = true;

            [net,tr] = train(net,X,Y);
            Yhat = net(X);

            % Identificacion explicita de la particion de cada muestra.
            partition = strings(nSamples,1);
            partition(tr.trainInd) = "Entrenamiento";
            partition(tr.valInd) = "Validacion";
            partition(tr.testInd) = "Prueba";

            % Metricas por particion y para el conjunto completo.
            setNames = ["Entrenamiento";"Validacion";"Prueba";"Total"];
            indices = {tr.trainInd,tr.valInd,tr.testInd,1:nSamples};
            MAE = zeros(4,1); RMSE = zeros(4,1); R2 = zeros(4,1);
            N = zeros(4,1);
            for q = 1:4
                idx = indices{q};
                yt = Y(idx); yp = Yhat(idx);
                err = yt-yp;
                N(q) = numel(idx);
                MAE(q) = mean(abs(err));
                RMSE(q) = sqrt(mean(err.^2));
                ssRes = sum(err.^2);
                ssTot = sum((yt-mean(yt)).^2);
                R2(q) = 1-ssRes/max(ssTot,eps);
            end

            metricsTable = table(setNames,N,MAE,RMSE,R2, ...
                'VariableNames',{'Conjunto','Muestras','MAE','RMSE','R2'});

            predictionTable = table((1:nSamples)',x,y,humidity,Yhat', ...
                (humidity-Yhat'),abs(humidity-Yhat'),partition, ...
                'VariableNames',{'SampleID','X_m','Y_m','Humedad_real', ...
                'Humedad_predicha','Error','Error_absoluto','Particion'});

            configTable = table( ...
                ["Tipo_modelo";"Entradas";"Salida";"Capas_ocultas"; ...
                 "Funcion_capa_1";"Funcion_capa_2";"Funcion_salida"; ...
                 "Algoritmo_entrenamiento";"Funcion_perdida"; ...
                 "Proporcion_entrenamiento";"Proporcion_validacion"; ...
                 "Proporcion_prueba";"Epocas_maximas";"Max_fail"; ...
                 "Semilla_aleatoria";"Campo";"Muestras_totales"], ...
                ["Regresion espacial";"X_m, Y_m";"Humedad normalizada"; ...
                 "16, 8";"tansig";"tansig";"purelin";"trainlm";"mse"; ...
                 "0.70";"0.15";"0.15";"1000";"20";"42"; ...
                 string(S.fieldType);string(nSamples)], ...
                'VariableNames',{'Parametro','Valor'});

            % Exportacion de matrices aprendidas. Cada fila corresponde a
            % una neurona de destino y cada columna a una entrada de origen.
            W1 = array2table(net.IW{1,1});
            W1.Properties.VariableNames = compose('Entrada_%d',1:size(net.IW{1,1},2));
            W1 = addvars(W1,(1:height(W1))','Before',1,'NewVariableNames','Neurona');

            W2 = array2table(net.LW{2,1});
            W2.Properties.VariableNames = compose('Neurona_origen_%d',1:size(net.LW{2,1},2));
            W2 = addvars(W2,(1:height(W2))','Before',1,'NewVariableNames','Neurona_destino');

            W3 = array2table(net.LW{3,2});
            W3.Properties.VariableNames = compose('Neurona_origen_%d',1:size(net.LW{3,2},2));
            W3 = addvars(W3,(1:height(W3))','Before',1,'NewVariableNames','Salida');

            biasTable = table( ...
                [repmat("Capa_1",numel(net.b{1}),1); ...
                 repmat("Capa_2",numel(net.b{2}),1); ...
                 repmat("Salida",numel(net.b{3}),1)], ...
                [(1:numel(net.b{1}))';(1:numel(net.b{2}))';(1:numel(net.b{3}))'], ...
                [net.b{1};net.b{2};net.b{3}], ...
                'VariableNames',{'Capa','Neurona','Sesgo'});

            epoch = (0:numel(tr.perf)-1)';
            trainingTable = table(epoch,tr.perf(:),tr.vperf(:),tr.tperf(:), ...
                'VariableNames',{'Epoca','MSE_entrenamiento','MSE_validacion','MSE_prueba'});

            if isfile(filename)
                delete(filename);
            end
            writetable(configTable,filename,'Sheet','Configuracion');
            writetable(metricsTable,filename,'Sheet','Metricas');
            writetable(predictionTable,filename,'Sheet','Predicciones');
            writetable(trainingTable,filename,'Sheet','Historia_entrenamiento');
            writetable(W1,filename,'Sheet','Pesos_capa_1');
            writetable(W2,filename,'Sheet','Pesos_capa_2');
            writetable(W3,filename,'Sheet','Pesos_salida');
            writetable(biasTable,filename,'Sheet','Sesgos');

            % El archivo MAT conserva normalizaciones y propiedades que no
            % quedan completamente representadas en las hojas de Excel.
            [folder,base,~] = fileparts(filename);
            matFilename = fullfile(folder,[base '.mat']);
            save(matFilename,'net','tr','metricsTable','hiddenLayers');

            uialert(fig,sprintf(['RNA entrenada y exportada.\n' ...
                'MAE de prueba: %.5f\nRMSE de prueba: %.5f\nR2 de prueba: %.5f'], ...
                MAE(3),RMSE(3),R2(3)),'Exportacion finalizada');
        catch ME
            uialert(fig,ME.message,'Error entrenando o exportando la RNA');
        end
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
                string(S.robotShape), ...
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
                'VariableNames',{'Metodo','Morfologia','Campo','Iteraciones_configuradas', ...
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

        % Construccion de un patron para la RNA sin posicion global.
        % La proximidad se simula con la distancia a los limites del entorno.
        [proxFront,proxLeft,proxRight] = simulatedProximity(S.headPos,S.theta);
        S.humidityWindow = [hCurrent;S.humidityWindow(1:end-1)];
        validHumidity = S.humidityWindow(~isnan(S.humidityWindow));
        hLag1 = hCurrent; hLag2 = hCurrent;
        if numel(validHumidity) >= 2, hLag1 = S.humidityWindow(2); end
        if numel(validHumidity) >= 3, hLag2 = S.humidityWindow(3); end
        deltaH = hCurrent-hLag1;
        meanH = mean(validHumidity);
        slopeH = (hCurrent-validHumidity(end))/max(numel(validHumidity)-1,1);
        sourceSignal = max(0,hCurrent-max([proxFront proxLeft proxRight]));
        S.virtualSignal = (1-S.virtualDecay)*S.virtualSignal + sourceSignal;

        % Etiqueta generada por el controlador experto. La posicion y el
        % mapa pueden intervenir en el experto, pero nunca en las entradas.
        expertError = atan2(sin(desiredTheta-S.theta),cos(desiredTheta-S.theta));
        if max([proxFront proxLeft proxRight]) >= 0.85
            expertAction = 4; % evasion de seguridad
        elseif expertError > pi/12
            expertAction = 2; % giro a la izquierda
        elseif expertError < -pi/12
            expertAction = 3; % giro a la derecha
        else
            expertAction = 1; % avance
        end

        S.rnaFeatures(end+1,:) = [hCurrent hLag1 hLag2 deltaH meanH slopeH ...
            proxFront proxLeft proxRight S.virtualSignal S.previousAction];
        S.rnaLabels(end+1,1) = expertAction;
        S.rnaRunIDs(end+1,1) = S.rnaRunID;
        S.previousAction = expertAction;

        angularError = expertError;
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
            ['Morfologia: ',char(S.robotShape)]
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

        % Muestra el campo verdadero completo, sin atenuación.
        rgb = zeros([size(S.trueHumidity),3]);

        for r = 1:size(S.trueHumidity,1)
            for c = 1:size(S.trueHumidity,2)

                val = S.trueHumidity(r,c);
                val = min(max(val,0),1);

                idc = max(1,min(256,round(1 + val*255)));

                rgb(r,c,1) = cmap(idc,1);
                rgb(r,c,2) = cmap(idc,2);
                rgb(r,c,3) = cmap(idc,3);

            end
        end

    else

        % Las celdas no descubiertas permanecen grises.
        rgb = zeros([size(S.trueHumidity),3]) + ...
              S.grayUnknownLevel;

        H = S.discoveredHumidity;
        mask = S.discoveredMask;

        for r = 1:size(H,1)
            for c = 1:size(H,2)

                if mask(r,c)

                    val = H(r,c);
                    val = min(max(val,0),1);

                    idc = max(1,min(256,round(1 + val*255)));

                    rgb(r,c,1) = cmap(idc,1);
                    rgb(r,c,2) = cmap(idc,2);
                    rgb(r,c,3) = cmap(idc,3);

                end
            end
        end
    end
end

    function drawRobot()
        S.numModules = moduleSpinner.Value;
        S.robotShape = string(shapeDrop.Value);

        if ~isfield(S,'robotPatches') || length(S.robotPatches) ~= S.numModules || any(~isvalid(S.robotPatches))
            if isfield(S,'robotPatches')
                try
                    delete(S.robotPatches(isvalid(S.robotPatches)));
                catch
                end
            end
            S.robotPatches = gobjects(S.numModules,1);
            for ii = 1:S.numModules
                S.robotPatches(ii) = patch(ax,nan,nan,[0.75 0.75 0.75], ...
                    'EdgeColor','k','LineWidth',1.1);
            end
        end

        t = S.k*0.08;
        relCenters = morphologyRelativeCenters(S.numModules,S.robotShape,S.moduleLength,t, ...
            S.waveAmplitude,S.waveFrequency,S.phaseLag);

        R = [cos(S.theta) -sin(S.theta); sin(S.theta) cos(S.theta)];
        centers = (R*relCenters')' + S.headPos;

        % Orientacion aproximada por modulo. En cadena se conserva la onda
        % senoidal; en T, + y L se orienta cada modulo segun su brazo.
        localAngles = morphologyLocalAngles(relCenters,S.robotShape,t, ...
            S.waveAmplitude,S.waveFrequency,S.phaseLag);
        angles = S.theta + localAngles;

        for ii = 1:S.numModules
            [xv,yv] = moduleRectangle(centers(ii,:),angles(ii),S.moduleLength,S.moduleWidth);
            set(S.robotPatches(ii),'XData',xv,'YData',yv);
        end

        [linkX,linkY] = morphologyLinkData(centers,S.robotShape,S.moduleLength);
        set(S.moduleLine,'XData',linkX,'YData',linkY);
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
    function [pFront,pLeft,pRight] = simulatedProximity(pos,heading)
        % Proximidad normalizada [0,1] respecto a los limites del entorno.
        maxRange = 0.20;
        angles = [heading,heading+pi/2,heading-pi/2];
        values = zeros(1,3);
        for jj = 1:3
            direction = [cos(angles(jj)),sin(angles(jj))];
            candidates = inf(1,4);
            if direction(1) > 1e-9
                candidates(1) = (S.worldSizeX-pos(1))/direction(1);
            elseif direction(1) < -1e-9
                candidates(2) = (0-pos(1))/direction(1);
            end
            if direction(2) > 1e-9
                candidates(3) = (S.worldSizeY-pos(2))/direction(2);
            elseif direction(2) < -1e-9
                candidates(4) = (0-pos(2))/direction(2);
            end
            valid = candidates(candidates >= 0 & isfinite(candidates));
            if isempty(valid), distance = maxRange; else, distance = min(valid); end
            values(jj) = max(0,min(1,(maxRange-distance)/maxRange));
        end
        pFront = values(1); pLeft = values(2); pRight = values(3);
    end

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


function relCenters = morphologyRelativeCenters(numModules,robotShape,L,t,A,f,phaseLag)
% Devuelve centros relativos en coordenadas locales respecto a la cabeza.
% La cabeza siempre es el modulo 1 y se ubica en [0 0].

numModules = max(1,round(numModules));
relCenters = zeros(numModules,2);

switch string(robotShape)
    case "Cadena"
        for i = 2:numModules
            osc = A*sin(2*pi*f*t - (i-1)*phaseLag);
            relCenters(i,:) = relCenters(i-1,:) - L*[cos(osc), sin(osc)];
        end

    case "T"
        % Tronco hacia atras y barra transversal al final del tronco.
        stemCount = max(2,ceil(numModules/2));
        stemCount = min(stemCount,numModules);
        for i = 1:stemCount
            relCenters(i,:) = [-(i-1)*L, 0];
        end

        remaining = numModules - stemCount;
        anchor = relCenters(stemCount,:);
        side = 1;
        level = 1;
        idx = stemCount + 1;
        while idx <= numModules
            relCenters(idx,:) = anchor + [0, side*level*L];
            side = -side;
            if side == 1
                level = level + 1;
            end
            idx = idx + 1;
        end

    case "Cruz +"
        % Cabeza en el centro; brazos en cuatro direcciones.
        dirs = [-1 0; 1 0; 0 1; 0 -1];
        idx = 2;
        level = 1;
        while idx <= numModules
            for d = 1:4
                if idx > numModules
                    break;
                end
                relCenters(idx,:) = level*L*dirs(d,:);
                idx = idx + 1;
            end
            level = level + 1;
        end

    case "L"
        % Primer brazo hacia atras; segundo brazo hacia arriba desde el codo.
        arm1 = max(2,ceil(numModules/2));
        arm1 = min(arm1,numModules);
        for i = 1:arm1
            relCenters(i,:) = [-(i-1)*L, 0];
        end
        anchor = relCenters(arm1,:);
        idx = arm1 + 1;
        level = 1;
        while idx <= numModules
            relCenters(idx,:) = anchor + [0, level*L];
            level = level + 1;
            idx = idx + 1;
        end

    otherwise
        for i = 2:numModules
            relCenters(i,:) = relCenters(i-1,:) - L*[1,0];
        end
end
end

function localAngles = morphologyLocalAngles(relCenters,robotShape,t,A,f,phaseLag)
numModules = size(relCenters,1);
localAngles = zeros(numModules,1);

switch string(robotShape)
    case "Cadena"
        for i = 2:numModules
            localAngles(i) = A*sin(2*pi*f*t - (i-1)*phaseLag);
        end

    otherwise
        for i = 2:numModules
            v = relCenters(i,:);
            if norm(v) > eps
                localAngles(i) = atan2(v(2),v(1));
            else
                localAngles(i) = 0;
            end
        end
end
end


function [linkX,linkY] = morphologyLinkData(centers,robotShape,L)
% Genera segmentos de conexion entre modulos sin unir brazos no vecinos.
numModules = size(centers,1);

if numModules <= 1
    linkX = centers(:,1);
    linkY = centers(:,2);
    return;
end

pairs = [];

switch string(robotShape)
    case "Cadena"
        pairs = [(1:numModules-1)' (2:numModules)'];

    otherwise
        % Arbol de vecindad: cada modulo se conecta con el modulo previo mas
        % cercano, evitando lineas diagonales largas entre brazos diferentes.
        connected = 1;
        remaining = 2:numModules;
        while ~isempty(remaining)
            bestI = remaining(1);
            bestJ = connected(1);
            bestD = inf;

            for ii = remaining
                for jj = connected
                    d = norm(centers(ii,:) - centers(jj,:));
                    if d < bestD
                        bestD = d;
                        bestI = ii;
                        bestJ = jj;
                    end
                end
            end

            pairs(end+1,:) = [bestJ bestI]; %#ok<AGROW>
            connected(end+1) = bestI; %#ok<AGROW>
            remaining(remaining == bestI) = [];
        end
end

linkX = [];
linkY = [];
for k = 1:size(pairs,1)
    i = pairs(k,1);
    j = pairs(k,2);
    linkX = [linkX centers(i,1) centers(j,1) nan]; %#ok<AGROW>
    linkY = [linkY centers(i,2) centers(j,2) nan]; %#ok<AGROW>
end
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
