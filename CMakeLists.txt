cmake_minimum_required(VERSION 3.9)

project(IDKPhysics)

file(
    GLOB SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/IDKPhysics/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/IDKPhysics/body/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/IDKPhysics/shape/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/src/IDKPhysics/constraint/*.cpp
)


add_library(${PROJECT_NAME} OBJECT ${SOURCES})

set_target_properties(
    ${PROJECT_NAME} PROPERTIES
    POSITION_INDEPENDENT_CODE ON
)

target_link_libraries(
    ${PROJECT_NAME} PUBLIC
    libidk
)


install(
    FILES ${PUBLIC_HEADER}
    DESTINATION ${CMAKE_SOURCE_DIR}/build/include/${PROJECT_NAME}/
)

FILE(
    COPY ${CMAKE_CURRENT_SOURCE_DIR}/src/${PROJECT_NAME}
    DESTINATION ${IDK_OUTPUT_DIRECTORY}/include/.
)
