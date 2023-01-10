cmake_minimum_required(VERSION 3.9)
project(nvmpi VERSION 1.0.0 DESCRIPTION "nvidia multimedia api")

set(CMAKE_C_FLAGS“$ {CMAKE_C_FLAGS} -fPIC”)
set(CMAKE_CXX_FLAGS“$ {CMAKE_CXX_FLAGS} -fPIC”)
#set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link=/lib")
#set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link=/usr/lib/aarch64-linux-gnu")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link=/usr/lib/aarch64-linux-gnu/tegra")
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-rpath-link=/usr/local/cuda/lib64")

find_library(LIB_NVBUF nvbuf_utils PATHS /usr/lib/aarch64-linux-gnu/tegra)
find_library(LIB_V4L2 nvv4l2 PATHS /usr/lib/aarch64-linux-gnu/tegra)
#

find_package (Threads)
#find_library(LIB_DRM drm PATHS /usr/lib/aarch64-linux-gnu/tegra)
#find_library(LIB_EGL EGL PATHS /usr/lib/aarch64-linux-gnu/tegra)

function(get_linux_lsb_release_information)
    find_program(LSB_RELEASE_EXEC lsb_release)
    if(NOT LSB_RELEASE_EXEC)
        message(FATAL_ERROR "Could not detect lsb_release executable, can not gather required information")
    endif()

    execute_process(COMMAND "${LSB_RELEASE_EXEC}" --short --id OUTPUT_VARIABLE LSB_RELEASE_ID_SHORT OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND "${LSB_RELEASE_EXEC}" --short --release OUTPUT_VARIABLE LSB_RELEASE_VERSION_SHORT OUTPUT_STRIP_TRAILING_WHITESPACE)
    execute_process(COMMAND "${LSB_RELEASE_EXEC}" --short --codename OUTPUT_VARIABLE LSB_RELEASE_CODENAME_SHORT OUTPUT_STRIP_TRAILING_WHITESPACE)

    set(LSB_RELEASE_ID_SHORT "${LSB_RELEASE_ID_SHORT}" PARENT_SCOPE)
    set(LSB_RELEASE_VERSION_SHORT "${LSB_RELEASE_VERSION_SHORT}" PARENT_SCOPE)
    set(LSB_RELEASE_CODENAME_SHORT "${LSB_RELEASE_CODENAME_SHORT}" PARENT_SCOPE)
endfunction()


get_linux_lsb_release_information()

set(sourceFiles 
		nvmpi_dec.cpp
		nvmpi_enc.cpp
		NvBuffer.cpp		
		NvElement.cpp
		NvElementProfiler.cpp
		NvLogging.cpp
		NvV4l2Element.cpp
		NvV4l2ElementPlane.cpp
		NvVideoDecoder.cpp
		NvVideoEncoder.cpp)

if(LSB_RELEASE_VERSION_SHORT GREATER_EQUAL "20")
	set(JETPACK_VER 5)
	add_definitions( -DJETPACK_VER=5 )
	message(STATUS " - ubuntu version +20")
	list(APPEND sourceFiles NvBufSurface.cpp)
	find_library(LIB_NVBUFSURF nvbufsurftransform PATHS /usr/lib/aarch64-linux-gnu/tegra)	
	
	configure_file(nvmpi.pc.in nvmpi.pc @ONLY)
else()
	set(JETPACK_VER 4)
	add_definitions( -DJETPACK_VER=4 )
	message(STATUS " - ubuntu version <20")
	set(LIB_NVBUFSURF "")
	
	configure_file(nvmpiJP4.pc.in nvmpi.pc @ONLY)
endif()

add_library(nvmpi SHARED ${sourceFiles} )
add_library(nvmpi_static STATIC ${sourceFiles} )

#set_target_properties(nvmpi_static PROPERTIES OUTPUT_NAME nvmpi)
set_target_properties(nvmpi PROPERTIES VERSION ${PROJECT_VERSION})
set_target_properties(nvmpi PROPERTIES SOVERSION 1)
set_target_properties(nvmpi nvmpi_static PROPERTIES PUBLIC_HEADER nvmpi.h)
target_link_libraries(nvmpi PRIVATE ${CMAKE_THREAD_LIBS_INIT} ${LIB_NVBUF}  ${LIB_V4L2} ${LIB_NVBUFSURF})
target_include_directories(nvmpi PRIVATE /usr/src/jetson_multimedia_api/include)
target_include_directories(nvmpi PRIVATE /usr/local/cuda/include)
target_include_directories(nvmpi_static PRIVATE /usr/src/jetson_multimedia_api/include)
target_include_directories(nvmpi_static PRIVATE /usr/local/cuda/include)

include(GNUInstallDirs)
install(TARGETS nvmpi nvmpi_static
	    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
	    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
	        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(FILES ${CMAKE_BINARY_DIR}/nvmpi.pc
	    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/pkgconfig)
install(FILES ${CMAKE_BINARY_DIR}/nvmpi.pc
	    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig)
