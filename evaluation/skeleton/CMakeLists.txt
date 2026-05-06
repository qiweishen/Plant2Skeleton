cmake_minimum_required(VERSION 3.22...3.25)
project(SkeletonEvaluator LANGUAGES CXX)

set(SOURCES
    main.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} PUBLIC
        SkelSeg_lib
)
