cmake_minimum_required(VERSION 3.7.1)
project(SloppyHandJob)

set(OPENVR_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/openvr)

option(BUILD_SLOPPYHANDJOB "Build SloppyHandJob GUI" ON)
if(BUILD_SLOPPYHANDJOB AND WIN32)
    include(FetchContent)

    FetchContent_Declare(SDL2 GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
                         GIT_TAG release-2.30.0 GIT_SHALLOW TRUE)
    FetchContent_MakeAvailable(SDL2)

    FetchContent_Declare(imgui GIT_REPOSITORY https://github.com/ocornut/imgui.git
                         GIT_TAG v1.90.4 GIT_SHALLOW TRUE)
    FetchContent_GetProperties(imgui)
    if(NOT imgui_POPULATED)
        FetchContent_Populate(imgui)
    endif()
    set(IMGUI_DIR ${imgui_SOURCE_DIR})

    add_executable(SloppyHandJob WIN32
        src/main.cpp
        ${IMGUI_DIR}/imgui.cpp
        ${IMGUI_DIR}/imgui_draw.cpp
        ${IMGUI_DIR}/imgui_tables.cpp
        ${IMGUI_DIR}/imgui_widgets.cpp
        ${IMGUI_DIR}/backends/imgui_impl_sdl2.cpp
        ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
        src/shared_state.h
    )

    target_compile_definitions(SloppyHandJob PRIVATE IMGUI_IMPL_OPENGL_LOADER_GL3W)

    target_include_directories(SloppyHandJob PRIVATE
        ${IMGUI_DIR} ${IMGUI_DIR}/backends
        ${SDL2_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${OPENVR_LIB_DIR}/headers
    )

    target_link_libraries(SloppyHandJob PRIVATE
        SDL2::SDL2 SDL2::SDL2main
        opengl32 ws2_32
        ${OPENVR_LIB_DIR}/lib/win64/openvr_api.lib
    )

    add_custom_command(TARGET SloppyHandJob POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "$<TARGET_FILE:SDL2::SDL2>" "$<TARGET_FILE_DIR:SloppyHandJob>"
    )

    add_custom_command(TARGET SloppyHandJob POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${OPENVR_LIB_DIR}/bin/win64/openvr_api.dll"
        "$<TARGET_FILE_DIR:SloppyHandJob>"
    )
endif()