
cmake_minimum_required(VERSION 3.20)

project(project_moon LANGUAGES CXX)

#Стандарт C++20
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#Предупреждения для всех конфигураций

if(MSVC)
    #Строгие предупреждения и проверки MSVC
    add_compile_options(/W4 /WX /permissive- /sdl)
else()
    #Строгие предупреждения GCC и Clang
    add_compile_options(-Wall -Wextra -Werror -Wpedantic)
endif()

#Debug: отладочная сборка

if(MSVC)
    #Отключение оптимизации и проверки времени выполнения
    string(APPEND CMAKE_CXX_FLAGS_DEBUG " /Od /RTC1")
else()
    #Отключение оптимизации и отладочная информация
    string(APPEND CMAKE_CXX_FLAGS_DEBUG " -O0 -g")
endif()

#Release: оптимизированная сборка

if(MSVC)
    string(APPEND CMAKE_CXX_FLAGS_RELEASE " /O2")
else()
    string(APPEND CMAKE_CXX_FLAGS_RELEASE " -O3")
endif()

#Исполняемый файл

add_executable(app
    main.cpp
    imitation.cpp
    imitation.h
)
