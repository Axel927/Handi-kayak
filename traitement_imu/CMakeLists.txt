#Déclaration du projet
project(tp1-vision)
set(EXECUTABLE_OUTPUT_PATH bin/${CMAKE_BUILD_TYPE})
include_directories(include)
#Déclaration de l'exécutable
add_executable(obtentionConsigne src/obtentionConsigne.cpp)
#Configuration de l'édition de liens
#target_link_libraries(hello pthread X11)
