cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RELOJ)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)


# --- BLOQUE LIBRERIAS AUTOMÁTICAS ---

#sino funciona ejecutar comando  CMake: Delete Cache and Reconfigure

#opciones para librerias
# BUSCAR LA CARPETA "librerias" EN CORE, CORE/SRC O EN LA RAÍZ DEL PROYECTO (Insensible a mayúsculas)
# La búsqueda en la raíz cubre el caso en que CubeMX no genera la
# estructura Core/Src (por ejemplo, al elegir el toolchain "CMake"),
# dejando los archivos generados directamente en la carpeta del proyecto.

set(RUTAS_DE_BUSQUEDA 
    "${CMAKE_SOURCE_DIR}/Core"
    "${CMAKE_SOURCE_DIR}/Core/Src"
    "${CMAKE_SOURCE_DIR}"
)

set(LIBRERIAS_DIR "")

#opciones para  carpeta de librerias
# Recorremos cada ruta posible (Core, Core/Src y finalmente la raíz del proyecto)
foreach(ruta ${RUTAS_DE_BUSQUEDA})
    # Solo buscamos si la ruta base existe y aún no hemos encontrado la carpeta
    if(EXISTS "${ruta}" AND NOT LIBRERIAS_DIR)
  
        file(GLOB contenido RELATIVE "${ruta}" "${ruta}/*")
        
        # Buscamos la carpeta ignorando mayúsculas/minúsculas
        foreach(item ${contenido})
            
            if(IS_DIRECTORY "${ruta}/${item}")
                string(TOLOWER "${item}" item_lower)
                if(item_lower STREQUAL "librerias")
                    set(LIBRERIAS_DIR "${ruta}/${item}")
                    break() # Encontramos la carpeta, salimos del bucle
                endif()
            endif()
        endforeach()
    endif()
endforeach()

# Mensaje para la consola de CMake y Fallback
if(LIBRERIAS_DIR)
    message(STATUS "Carpeta de librerías encontrada en: ${LIBRERIAS_DIR}")
else()
    message(WARNING "No se encontró la carpeta 'librerias' ni en Core/, Core/Src/ ni en la raíz del proyecto")
    # Fallback por defecto para evitar errores catastróficos en CMake
    set(LIBRERIAS_DIR "${CMAKE_SOURCE_DIR}/Core/librerias") 
endif()


#  INCLUIR FUENTES Y CABECERAS

# Buscar todos los .c en la carpeta encontrada y subcarpetas
file(GLOB_RECURSE LIBRERIAS_SOURCES CONFIGURE_DEPENDS
    "${LIBRERIAS_DIR}/*.c"
)
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
   ${LIBRERIAS_SOURCES}
)

# Buscar todos los .h en la carpeta encontrada y subcarpetas
file(GLOB_RECURSE LIBRERIAS_HEADERS CONFIGURE_DEPENDS
    "${LIBRERIAS_DIR}/*.h"
)
# Extraer las carpetas donde están esos .h
set(LIBRERIAS_INCLUDE_DIRS "")

foreach(header ${LIBRERIAS_HEADERS})
    get_filename_component(header_dir "${header}" DIRECTORY)
    list(APPEND LIBRERIAS_INCLUDE_DIRS "${header_dir}")
endforeach()

# Evitar errores si no hay headers
if(LIBRERIAS_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES LIBRERIAS_INCLUDE_DIRS)
endif()

target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${LIBRERIAS_INCLUDE_DIRS}
)
#fin librerias


# --- BLOQUE SOPORTE FLOAT Y MATH (-lm) ---

# Habilitar soporte de float en printf/sprintf/scanf y vincular math.h (-lm)
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE
    -u _printf_float
    -lm
)


# --- BLOQUE GENERACIÓN HEX Y BIN ---

# Generar .hex y .bin automáticamente después de compilar
add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${CMAKE_PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generando ${CMAKE_PROJECT_NAME}.hex y ${CMAKE_PROJECT_NAME}.bin"
)


# --- BLOQUE OPTIMIZACIÓN (-O2) ---

# Optimización según tipo de build
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -Og -g3)
elseif(CMAKE_BUILD_TYPE STREQUAL "Release")
    target_compile_options(${CMAKE_PROJECT_NAME} PRIVATE -O2 -g0)
endif()
