cmake_minimum_required(VERSION 3.20)
project(project_moon CXX)

#стандарт C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#жесткие флаги компилятора и оптимизация
add_compile_options(
        -O3
        -Wall
        -Werror
        -Wextra
)

#исполняемый файл "app" из исходника "main.cpp"
add_executable(app main.cpp
        imitation.cpp
        imitation.h)
