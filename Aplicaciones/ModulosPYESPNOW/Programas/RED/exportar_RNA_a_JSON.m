function exportar_RNA_a_JSON(archivoMAT, archivoJSON)


if nargin < 1, archivoMAT = 'modelo_RNA_navegacion.mat'; end
if nargin < 2, archivoJSON = 'modelo_RNA_navegacion.json'; end

S = load(archivoMAT);
if ~isfield(S,'net')
    error('El archivo MAT debe contener la variable net.');
end
net = S.net;

if numel(net.layers) ~= 4
    error('Se esperaba una red con cuatro capas: 32-16-8-4.');
end
if size(net.IW{1,1},2) ~= 11
    error('Se esperaban 11 variables de entrada.');
end

ps = net.inputs{1}.processSettings;
idx = find(strcmp(net.inputs{1}.processFcns,'mapminmax'),1);
if isempty(idx)
    error('No se encontró el preprocesamiento mapminmax.');
end
mm = ps{idx};

out.architecture = [11 32 16 8 4];
out.classes = {'AV','GI','GD','EV'};
out.input_process.xoffset = reshape(mm.xoffset,1,[]);
out.input_process.gain = reshape(mm.gain,1,[]);
out.input_process.ymin = repmat(mm.ymin,1,11);

out.layers = cell(1,4);
out.layers{1}.W = net.IW{1,1};
out.layers{1}.b = reshape(net.b{1},1,[]);
for k = 2:4
    out.layers{k}.W = net.LW{k,k-1};
    out.layers{k}.b = reshape(net.b{k},1,[]);
end

texto = jsonencode(out);
fid = fopen(archivoJSON,'w');
if fid < 0, error('No fue posible crear %s.',archivoJSON); end
limpieza = onCleanup(@() fclose(fid));
fwrite(fid,texto,'char');
fprintf('Modelo exportado: %s\n',archivoJSON);
end
