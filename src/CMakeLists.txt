
file(GLOB_RECURSE USER_SOURCES
        ${CMAKE_CURRENT_LIST_DIR}/*.c
        ${CMAKE_CURRENT_LIST_DIR}/*.cc
        ${CMAKE_CURRENT_LIST_DIR}/*.cpp
        ${CMAKE_CURRENT_LIST_DIR}/*.s
        ${CMAKE_CURRENT_LIST_DIR}/*.S)

target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        ${USER_SOURCES}
)
target_include_directories(stm32cubemx INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
)