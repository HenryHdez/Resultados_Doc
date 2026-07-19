function Entrenamiento(carpetaDatos)
clc;

if nargin < 1 || strlength(string(carpetaDatos)) == 0
    carpetaDatos = uigetdir(pwd,'Seleccione la carpeta con los Excel');
    if isequal(carpetaDatos,0), return; end
end

if exist('patternnet','file') ~= 2
    error('Se requiere Deep Learning Toolbox: no se encontro patternnet.');
end

rng(42,'twister');
archivos = dir(fullfile(carpetaDatos,'*Corridas*.xlsx'));
if isempty(archivos)
    error('No se encontraron archivos *Corridas*.xlsx.');
end

entradas = {'Humedad_k','Humedad_k_1','Humedad_k_2', ...
    'Delta_humedad','Promedio_humedad','Pendiente_humedad', ...
    'Proximidad_frente','Proximidad_izquierda','Proximidad_derecha', ...
    'Senal_virtual_C','Accion_anterior'};
nombresClases = ["Avanzar";"Izquierda";"Derecha";"Evasion"];

datos = table();
contadorGlobal = 0;

for a = 1:numel(archivos)
    ruta = fullfile(archivos(a).folder,archivos(a).name);
    T = readtable(ruta,'Sheet','Datos_brutos','VariableNamingRule','preserve');
    requeridas = [entradas,{'RunID','Accion_experta'}];
    faltantes = setdiff(requeridas,T.Properties.VariableNames);
    if ~isempty(faltantes)
        error('En %s faltan columnas: %s',archivos(a).name,strjoin(faltantes,', '));
    end

    nombre = lower(string(archivos(a).name));
    if contains(nombre,'serpiente')
        morfologia = "Serpiente"; modulos = 6;
    elseif contains(nombre,'cruz')
        morfologia = "Cruz"; modulos = 5;
    elseif contains(nombre,'ele')
        morfologia = "L"; modulos = 5;
    elseif contains(nombre,'te_') || contains(nombre,'_te')
        morfologia = "T"; modulos = 5;
    else
        morfologia = "No_identificada"; modulos = NaN;
    end

    runsLocales = unique(T.RunID,'stable');
    runGlobal = zeros(height(T),1);
    for r = 1:numel(runsLocales)
        contadorGlobal = contadorGlobal+1;
        runGlobal(T.RunID==runsLocales(r)) = contadorGlobal;
    end
    T.RunID_Global = runGlobal;
    T.Morfologia = repmat(morfologia,height(T),1);
    T.Modulos = repmat(modulos,height(T),1);
    T.Archivo_origen = repmat(string(archivos(a).name),height(T),1);
    datos = [datos;T]; %#ok<AGROW>
end

% Verificaciones basicas.
Xfilas = datos{:,entradas};
Yclase = double(datos.Accion_experta);
if any(~isfinite(Xfilas),'all') || any(~isfinite(Yclase))
    error('Los datos contienen valores NaN o infinitos.');
end
if any(~ismember(Yclase,1:4))
    error('Accion_experta debe contener solamente las clases 1, 2, 3 y 4.');
end

% Division estratificada por morfologia y por corridas completas.
% Cada morfologia queda representada en los tres conjuntos.
morfologias = unique(datos.Morfologia,'stable');
runsTrain = []; runsVal = []; runsTest = [];
resumenParticion = table();
for m = 1:numel(morfologias)
    idxMorph = datos.Morfologia==morfologias(m);
    runsMorph = unique(datos.RunID_Global(idxMorph),'stable');
    if numel(runsMorph) < 3
        error('La morfologia %s requiere al menos tres corridas.',morfologias(m));
    end
    runsMorph = runsMorph(randperm(numel(runsMorph)));

    % Distribucion definida para los conjuntos actualmente recolectados.
    if numel(runsMorph)==12
        nTrainMorph=8; nValMorph=2; nTestMorph=2;
    elseif numel(runsMorph)==24
        nTrainMorph=18; nValMorph=3; nTestMorph=3;
    else
        nValMorph=max(1,round(0.15*numel(runsMorph)));
        nTestMorph=max(1,round(0.15*numel(runsMorph)));
        nTrainMorph=numel(runsMorph)-nValMorph-nTestMorph;
    end
    if nTrainMorph<1
        error('No fue posible dividir la morfologia %s.',morfologias(m));
    end

    rTrain=runsMorph(1:nTrainMorph);
    rVal=runsMorph(nTrainMorph+1:nTrainMorph+nValMorph);
    rTest=runsMorph(nTrainMorph+nValMorph+1:nTrainMorph+nValMorph+nTestMorph);
    runsTrain=[runsTrain;rTrain(:)]; %#ok<AGROW>
    runsVal=[runsVal;rVal(:)]; %#ok<AGROW>
    runsTest=[runsTest;rTest(:)]; %#ok<AGROW>

    resumenParticion=[resumenParticion;table(morfologias(m), ...
        numel(rTrain),numel(rVal),numel(rTest), ...
        'VariableNames',{'Morfologia','Corridas_entrenamiento', ...
        'Corridas_validacion','Corridas_prueba'})]; %#ok<AGROW>
end

idxTrain = find(ismember(datos.RunID_Global,runsTrain))';
idxVal = find(ismember(datos.RunID_Global,runsVal))';
idxTest = find(ismember(datos.RunID_Global,runsTest))';

X = Xfilas';
targets = full(ind2vec(Yclase',4));

% Arquitectura 11-32-16-8-4.
net = patternnet([32 16 8],'trainscg','crossentropy');
net.name = 'RNA_navegacion_sensorial';
net.inputs{1}.processFcns = {'removeconstantrows','mapminmax'};
net.layers{1}.transferFcn = 'tansig';
net.layers{2}.transferFcn = 'tansig';
net.layers{3}.transferFcn = 'tansig';
net.layers{4}.transferFcn = 'softmax';
net.divideFcn = 'divideind';
net.divideParam.trainInd = idxTrain;
net.divideParam.valInd = idxVal;
net.divideParam.testInd = idxTest;
net.trainParam.epochs = 500;
net.trainParam.max_fail = 20;
net.trainParam.showWindow = true;

[net,tr] = train(net,X,targets);
probabilidades = net(X);
[confianza,predicha] = max(probabilidades,[],1);

particion = strings(height(datos),1);
particion(idxTrain) = "Entrenamiento";
particion(idxVal) = "Validacion";
particion(idxTest) = "Prueba";
datos.Particion = particion;

% Metricas globales por conjunto.
conjuntos = ["Entrenamiento";"Validacion";"Prueba";"Total"];
indices = {idxTrain,idxVal,idxTest,1:height(datos)};
nMuestras=zeros(4,1); exactitud=zeros(4,1); exactitudBalanceada=zeros(4,1);
for s = 1:4
    id = indices{s}; nMuestras(s)=numel(id);
    exactitud(s)=mean(predicha(id)'==Yclase(id));
    recall=nan(4,1);
    for c=1:4
        idc=id(Yclase(id)==c);
        if ~isempty(idc), recall(c)=mean(predicha(idc)'==Yclase(idc)); end
    end
    exactitudBalanceada(s)=mean(recall,'omitnan');
end
metricas = table(conjuntos,nMuestras,exactitud,exactitudBalanceada, ...
    'VariableNames',{'Conjunto','Muestras','Exactitud','Exactitud_balanceada'});

% Precision, sensibilidad y F1 por clase en prueba.
CM = confusionmat(Yclase(idxTest),predicha(idxTest)','Order',1:4);
TP=diag(CM); precision=TP./max(sum(CM,1)',1);
sensibilidad=TP./max(sum(CM,2),1);
F1=2*(precision.*sensibilidad)./max(precision+sensibilidad,eps);
metricasClase = table((1:4)',nombresClases,precision,sensibilidad,F1, ...
    'VariableNames',{'Clase','Accion','Precision','Sensibilidad','F1'});

% Tabla de predicciones.
predicciones = datos(:,{'RunID_Global','Morfologia','Modulos','Accion_experta'});
predicciones.Particion = particion;
predicciones.Accion_predicha = predicha';
predicciones.Confianza = confianza';
for c=1:4
    predicciones.("P_"+nombresClases(c)) = probabilidades(c,:)';
end

% Parametros de preprocesamiento requeridos para Python.
ps = net.inputs{1}.processSettings;
mapIndex = find(strcmp(net.inputs{1}.processFcns,'mapminmax'),1);
mapSettings = ps{mapIndex};

modelo = struct();
modelo.nombre = net.name;
modelo.entradas = entradas;
modelo.clases = cellstr(nombresClases);
modelo.activaciones = {'tansig','tansig','tansig','softmax'};
modelo.W1 = net.IW{1,1}; modelo.b1 = net.b{1};
modelo.W2 = net.LW{2,1}; modelo.b2 = net.b{2};
modelo.W3 = net.LW{3,2}; modelo.b3 = net.b{3};
modelo.W4 = net.LW{4,3}; modelo.b4 = net.b{4};
modelo.mapminmax.xoffset = mapSettings.xoffset;
modelo.mapminmax.gain = mapSettings.gain;
modelo.mapminmax.ymin = mapSettings.ymin;
modelo.mapminmax.xrows = mapSettings.xrows;

carpetaSalida = fullfile(carpetaDatos,'salida_RNA');
if ~exist(carpetaSalida,'dir'), mkdir(carpetaSalida); end
rutaMAT = fullfile(carpetaSalida,'modelo_RNA_navegacion.mat');
rutaJSON = fullfile(carpetaSalida,'modelo_RNA_navegacion.json');
rutaXLSX = fullfile(carpetaSalida,'resultados_RNA_navegacion.xlsx');
rutaConsolidado = fullfile(carpetaSalida,'consolidado_RNA_estratificado.xlsx');

save(rutaMAT,'net','tr','modelo','metricas','metricasClase', ...
    'runsTrain','runsVal','runsTest');
fid=fopen(rutaJSON,'w');
if fid<0, error('No fue posible crear el archivo JSON.'); end
fprintf(fid,'%s',jsonencode(modelo,'PrettyPrint',true));
fclose(fid);

if isfile(rutaXLSX), delete(rutaXLSX); end
writetable(metricas,rutaXLSX,'Sheet','Metricas');
writetable(metricasClase,rutaXLSX,'Sheet','Metricas_clase_prueba');
writetable(predicciones,rutaXLSX,'Sheet','Predicciones');
writetable(array2table(CM,'VariableNames',cellstr("Pred_"+nombresClases)), ...
    rutaXLSX,'Sheet','Matriz_confusion');
writematrix(modelo.W1,rutaXLSX,'Sheet','Pesos_capa_1');
writematrix(modelo.W2,rutaXLSX,'Sheet','Pesos_capa_2');
writematrix(modelo.W3,rutaXLSX,'Sheet','Pesos_capa_3');
writematrix(modelo.W4,rutaXLSX,'Sheet','Pesos_salida');

% Consolidado independiente para auditar y reutilizar el entrenamiento.
if isfile(rutaConsolidado), delete(rutaConsolidado); end
writetable(datos,rutaConsolidado,'Sheet','Datos_consolidados');
writetable(resumenParticion,rutaConsolidado,'Sheet','Particion_morfologia');
asignacionRuns=table([runsTrain;runsVal;runsTest], ...
    [repmat("Entrenamiento",numel(runsTrain),1); ...
     repmat("Validacion",numel(runsVal),1); ...
     repmat("Prueba",numel(runsTest),1)], ...
    'VariableNames',{'RunID_Global','Particion'});
runMetadata=unique(datos(:,{'RunID_Global','Morfologia','Modulos','Archivo_origen'}), ...
    'rows','stable');
asignacionRuns=outerjoin(asignacionRuns,runMetadata,'Keys','RunID_Global', ...
    'MergeKeys',true,'Type','left');
writetable(asignacionRuns,rutaConsolidado,'Sheet','Asignacion_corridas');

fprintf('\nEntrenamiento finalizado.\n');
disp(metricas);
disp(metricasClase);
fprintf('Modelo MATLAB: %s\n',rutaMAT);
fprintf('Modelo Python: %s\n',rutaJSON);
fprintf('Resultados: %s\n',rutaXLSX);
fprintf('Consolidado estratificado: %s\n',rutaConsolidado);
end
