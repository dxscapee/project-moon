cmake_minimum_required(VERSION 3.20)
project(project_moon CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(MSVC)
    # флаги для компилятора Visual Studio
    add_compile_options(/W4 /WX /O2)
else()
    # флаги для GCC и Clang (macOS, Linux)
    add_compile_options(-O3 -Wall -Werror -Wextra)
endif()

add_executable(app main.cpp
        imitation.cpp
        imitation.h)