cmake_minimum_required(VERSION 3.16)

# Usa o componente da raiz deste repositório. EXTRA_COMPONENT_DIRS registra o
# componente pelo nome da pasta clonada; como o main não declara REQUIRES, ele
# já depende de todos os componentes do build, então o nome não importa aqui.
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../..")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(a7672sa_basic)
