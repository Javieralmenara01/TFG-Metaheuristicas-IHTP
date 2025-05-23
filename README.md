# TFG - Metaheurísticas en la optimización de la gestión hospitalaria

## Descripción

Este repositorio contiene el código fuente de mi Trabajo de Fin de Grado (TFG) _Metaheurísticas en la optimización de la gestión hospitalaria_, cuyo objetivo es la implementación de un algoritmo de optimización para el problema IHTP (Integrated Healthcare Timetabling Problem) presentado en la competición [IHTC](https://ihtc2024.github.io), mediente el uso de un algoritmo evolutivo.

El proyecto está implementado principalmente en C++ y utiliza JSON para la entrada/salida de datos. Contamos con diversos directorios, _Codificación A - Pacientes_ y _Codificación B - Pacientes y Recursos_, que representan diferentes enfoques para resolver el problema. Asimismo, se incluye el código de la primera aproximación al problema, que se trata de una búsqueda aleatoria constructiva _RandomSearch_.

## Estructura de directorios

```text
├── Codificación A - Pacientes                 # Implementación de la codificación A 
|   ├── src/                                      # Código fuente principal en C++
│   ├── main.cc                                   # Archivo principal
│   └── CMakeLists.txt                            # Archivo de configuración de CMake
├── Codificación B - Pacientes y enfermeros    # Implementación de la codificación B  
|   ├── src/                                      # Código fuente principal en C++
│   ├── main.cc                                   # Archivo principal
│   └── CMakeLists.txt                            # Archivo de configuración de CMake
├── instances                                  # Instancias del problema en formato JSON
│   ├── ihtc2024_copetition_instances/             # Instancias públicas de la competición 
│   ├── ihtc2024_final_instances/                  # Instancias "ocultas" de la competición
│   ├── ihtc2024_test_instances/                   # Instancias de prueba
│   └── ihtc2024_test_solutions/                   # Soluciones para las instancias de prueba
├── RandomSearch                               # Implementación de la búsqueda aleatoria
│   ├── src/                                      # Código fuente principal en C++ 
│   ├── main.cc                                   # Archivo principal
│   └── CMakeLists.txt                            # Archivo de configuración de CMake
├── validator                                  # Validador de soluciones
│   ├── IHTP_Validator.cc                         # Código fuente del validador
│   └── json.hpp                                  # Archivo de cabecera de la biblioteca JSON
├── .gitignore                                 # Ignorar archivos innecesarios
└── README.md                                  # Este archivo
```

## Ficheros fuente
El código fuente está organizado en diferentes directorios según la codificación utilizada. Cada directorio contiene un archivo `main.cc` que sirve como punto de entrada para el programa, así como un archivo `CMakeLists.txt` para gestionar la compilación.
El directorio `src` contiene los archivos de implementación de las clases y funciones necesarias para resolver el problema. La estructura de los archivos es la siguiente:
```text
├── src/
│   ├── Functions.cc                            # Funciones auxiliares
│   ├── Functions.h                             # Cabecera de las funciones auxiliares
│   ├── ProblemInstance.cc                      # Clase que representa la instancia del problema
│   ├── ProblemInstance.h                       # Cabecera de la clase ProblemInstance
│   ├── RandomSolver.cc                         # Clase que implementa la búsqueda aleatoria
│   ├── RandomSolver.h                          # Cabecera de la clase RandomSolver
│   ├── MetaHeuristicSolver.cc                  # Clase que implementa el algoritmo evolutivo
│   ├── MetaHeuristicSolver.h                   # Cabecera de la clase MetaHeuristicSolver
│   ├── Solver.cc                               # Clase base para el resolutor
│   ├── Solver.h                                # Cabecera de la clase Solver
│   └── Structs.h                               # Estructuras de datos utilizadas en el proyecto
```


## Dependencias

- C++: estandar C++17 o superior
- CMake: versión 3.15 o superior
- JSON: biblioteca [nlohmann/json](https://github.com/nlohmann/json)

### Instalación de dependencias

Cada directorio de codificación tiene su propio archivo `CMakeLists.txt` para gestionar las dependencias y la compilación. Asegúrate de tener CMake instalado en tu sistema.

Mi archivo `CMakeLists.txt` está configurado para usar la biblioteca JSON de nlohmann. Usando vcpkg es la siguiente:

```cmake
# CMakeLists.txt
# ================================
cmake_minimum_required(VERSION 3.15)
project(ProyectoTFG LANGUAGES CXX)

# ================================
# Usa C++17 en todo el proyecto
set(CMAKE_CXX_STANDARD         17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)
# ================================

set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-fsanitize=address -g)
add_link_options   (-fsanitize=address)


# 1) Por defecto, Release si no piden otro build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# 2) Reemplaza -Ofast por -O3 -ffast-math en Release
#    (quita cualquier anterior -Ofast que estuvieras inyectando)
set(CMAKE_C_FLAGS_RELEASE   "-O3 -ffast-math ${CMAKE_C_FLAGS_RELEASE} -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -ffast-math ${CMAKE_CXX_FLAGS_RELEASE} -g")

# 3) Opcional: silencia el warning de infinito de nlohmann/json
add_compile_options(-Wno-nan-infinity-disabled)

# 4) Añade tu ejecutable
# IMPORTANTE: Dependiendo de la codificación que uses se usaran unos archivos u otros que debes modificar según las necesidades
add_executable(main
  main.cc
  src/Functions.cc
  src/ProblemInstance.cc
  src/RandomSolver.cc
  src/MetaHeuristicSolver.cc
  src/Solver.cc
)

# 5) Marca el include de vcpkg como SYSTEM para evitar warnings en sus headers
target_include_directories(main SYSTEM PRIVATE 
# IMPORTANTE : Cambia [osx/linux] por tu sistema operativo
  ${CMAKE_SOURCE_DIR}/vcpkg/installed/x64-[osx/linux]/include
)
```

#### Instalación de vcpkg

```bash
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh             # Unix/macOS
# o bootstrap-vcpkg.bat en Windows
./vcpkg install nlohmann-json
```

## Compilación

Para compilar el proyecto, asegúrate de tener CMake instalado y ejecuta los siguientes comandos en la raíz del proyecto:

```bash
mkdir build && cd build
cmake ..
cmake --build .
```

## Ejecución

Una vez compilado, puedes ejecutar el programa desde la carpeta `build`:

Para los archivos de codificación A o B, ejecuta el siguiente comando: (Si lo deseas puedes cambiar el nombre del ejecutable)
```bash
./main <ruta_al_archivo_json> <ruta_de_salida> <population_size> <num_elites> <crossoverProb> <maxDuration>
```

Donde `<ruta_al_archivo_json>` es la ruta al archivo JSON de entrada y `<ruta_de_salida>` es la ruta donde se guardará el resultado en formato JSON. Los parámetros del algoritmo son: <population_size> (tamaño de la población), <num_elites> (número de élites), <crossoverProb> (probabilidad de cruce) y <maxDuration> (duración máxima en segundos).

Para la búsqueda aleatoria, ejecuta el siguiente comando:

```bash
./main <ruta_al_archivo_json> <ruta_de_salida> <maxDuration>
```
Donde `<ruta_al_archivo_json>` es la ruta al archivo JSON de entrada y `<ruta_de_salida>` es la ruta donde se guardará el resultado en formato JSON. El parámetro `<maxDuration>` es la duración máxima en minutos.

## Validador

El validador se encuentra en la carpeta `validator`. Para usarlo, ejecuta el siguiente comando:

Compilalo usando g++ u otro compilador de C++:

```bash
g++ -o validator validator/validator.cc -o IHTP_Validator
```

Luego ejecuta el validador con el siguiente comando:

```bash
./IHTP_Validator <ruta_a_la_instancia> <ruta_a_la_solucion>
```

Donde `<ruta_a_la_instancia>` es la ruta al archivo JSON de entrada y `<ruta_a_la_solucion>` es la ruta al archivo JSON que quieres validar.

## Contribuciones

Si deseas contribuir al proyecto, no dudes en abrir un issue o un pull request. Todas las contribuciones son bienvenidas.

## Licencia

Este proyecto está bajo la Licencia MIT. Consulta el archivo [LICENSE](LICENSE) para más detalles.

## Contacto

Si tienes alguna pregunta o comentario, no dudes en contactarme a través de email: [alu0101466552@ull.edu.es](mailto:alu0101466552@ull.edu.es) o en [LinkedIn](https://www.linkedin.com/in/javier-almenara-herrera)
