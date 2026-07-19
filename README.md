# Datos y aplicaciones de la tesis doctoral

Este repositorio reúne los datos experimentales, aplicaciones y archivos asociados con la tesis doctoral sobre coordinación y control de un robot modular mediante aprendizaje computacional y comunicación bioinspirada.

El propósito del repositorio es facilitar la consulta, organización y reproducción de los experimentos realizados con diferentes métodos de navegación y morfologías del robot.

## Estructura del repositorio

```text
.
├── Aplicaciones/   Aplicaciones y herramientas utilizadas en los experimentos
├── Mediciones/     Registros de funcionamiento y comunicación del robot
└── RNA/            Archivos relacionados con la red neuronal artificial
```

### Aplicaciones

Contiene las herramientas empleadas para configurar las pruebas, ejecutar los métodos de navegación, supervisar el robot y almacenar los resultados.

### Mediciones

Incluye los datos obtenidos durante las corridas experimentales. Los archivos están organizados por método y morfología del robot e incorporan métricas de navegación, movimiento, humedad y comunicación.

Entre los métodos evaluados se encuentran:

- red neuronal artificial (RNA);
- algoritmo genético;
- ascenso a la colina;
- exploración de frontera;
- quimiosíntesis bacteriana;
- recocido simulado;
- caminata aleatoria.

Las morfologías consideradas incluyen configuraciones en cruz, L, T y serpiente.

Los archivos identificados con el sufijo `_COM` contienen los registros de comunicación, como periodo entre paquetes, frecuencia efectiva, jitter, throughput y RSSI.

### RNA

Contiene los archivos utilizados para el entrenamiento, validación, prueba e implementación de la red neuronal artificial empleada en la navegación del robot modular.

## Formatos

Los datos se distribuyen principalmente en archivos:

- `.xlsx`: resultados y resúmenes de las corridas;
- `.json`: parámetros de configuración de los experimentos;
- `.m`: aplicaciones y rutinas desarrolladas en MATLAB;
- `.mat`: modelos y variables almacenadas por MATLAB.

## Uso de los datos

Los archivos se publican con fines académicos y de reproducibilidad. Al reutilizar los datos, se recomienda conservar los nombres de las variables, identificar el método y la morfología analizados, y citar la tesis y este repositorio.

## Autor

**Henry Alberto Hernández Martínez**  
Candidato a doctor en Ingeniería  
Universidad Distrital Francisco José de Caldas, Colombia

## Citación

La referencia bibliográfica completa de la tesis y el identificador permanente del repositorio se incorporarán cuando estén disponibles.

## Licencia

La licencia de uso del código y de los datos está pendiente de definición.
