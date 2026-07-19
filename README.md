# Introdución al repositorio

Este repositorio reúne los datos experimentales, aplicaciones y archivos derivados de la tesis doctoral titulada "Método para coordinar y controlar los movimientos de un robot modular tipo cadena en un entorno no estructurado y dinámico utilizando un algoritmo de aprendizaje computacional y un mecanismo de comunicación bio-inspirado".

El propósito del repositorio es facilitar la consulta, organización y reproducción de los experimentos realizados con diferentes métodos de navegación y morfologías de un robot modular tipo cadena (caso de estudio -Robot EMeRGE-).

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

Si utiliza los datos, aplicaciones o código de este repositorio, cite:

> Hernández Martínez, H. A. (2026). *Resultados_Doc: Datos y aplicaciones de la tesis doctoral* [Conjunto de datos y código fuente]. GitHub. https://github.com/HenryHdez/Resultados_Doc

### BibTeX

```bibtex
@misc{hernandez2026resultados,
  author       = {Henry Alberto Hernández Martínez},
  title        = {Resultados_Doc: Datos y aplicaciones de la tesis doctoral},
  year         = {2026},
  publisher    = {GitHub},
  howpublished = {Repositorio de datos y código fuente},
  url          = {https://github.com/HenryHdez/Resultados_Doc}
}
```

## Licencia

Este repositorio utiliza licencias diferenciadas según el tipo de contenido:

- Los datos experimentales, mediciones, documentación y modelos entrenados se distribuyen bajo la licencia [Creative Commons Atribución 4.0 Internacional (CC BY 4.0)](https://creativecommons.org/licenses/by/4.0/).
- El código fuente y los scripts desarrollados para este proyecto se distribuyen bajo la [Licencia MIT](https://opensource.org/license/mit).

La reutilización de los datos requiere reconocer la autoría y citar este repositorio:

> Hernández Martínez, H. A. (2026). *Resultados_Doc: Datos y aplicaciones de la tesis doctoral* [Conjunto de datos y código fuente]. GitHub. https://github.com/HenryHdez/Resultados_Doc

Los componentes de terceros conservan sus respectivas licencias y no quedan cubiertos por las licencias anteriores.
