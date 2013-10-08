function( _set_csm_dirs)

    set( CSM_PATCH_DIR ${CMAKE_CURRENT_SOURCE_DIR}/csm/patches PARENT_SCOPE )
    set( CSM_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}/csm PARENT_SCOPE )

endfunction( _set_csm_dirs)

function( download_and_compile_csm )
    _set_csm_dirs()

    include(ExternalProject)
    ExternalProject_Add(csm_src
        #--CMake variables ---------------------------------------------------------
        PREFIX ${CSM_BINARY_DIR}
        TMP_DIR ${CSM_BINARY_DIR}/tmp
        STAMP_DIR ${CSM_BINARY_DIR}/stamp
        SOURCE_DIR ${CSM_BINARY_DIR}/src
        #--Download step -----------------------------------------------------------
        GIT_REPOSITORY https://github.com/AndreaCensi/csm.git
        GIT_TAG 23703a998fff57250322
        #-- Update/Patch command ---------------------------------------------------
        PATCH_COMMAND patch -p0 -d ${CSM_BINARY_DIR}/src < ${CSM_PATCH_DIR}/patch_a && patch -p0 -d ${CSM_BINARY_DIR}/src < ${CSM_PATCH_DIR}/patch_b && patch -p0 -d ${CSM_BINARY_DIR}/src < ${CSM_PATCH_DIR}/patch_c && patch -p0 -d ${CSM_BINARY_DIR}/src < ${CSM_PATCH_DIR}/patch_d
        #-- Configure step ---------------------------------------------------------
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CSM_BINARY_DIR}/install
        #-- Build step -------------------------------------------------------------
        BUILD_IN_SOURCE 1
    )

    # Create phony target csm and add dependency on csm_source
    add_library(csm SHARED IMPORTED)
    set_target_properties(csm PROPERTIES IMPORTED_LOCATION "${CSM_BINARY_DIR}/install/lib/libcsm.so")
    add_dependencies(csm csm_src)

    # Pass variables to parent scope to interface with the new library
    set(csm_LIBRARIES csm PARENT_SCOPE)
    set(csm_INCLUDE_DIRS ${CSM_BINARY_DIR}/install/include PARENT_SCOPE)
    set(csm_EXPORTED_TARGETS csm PARENT_SCOPE)

endfunction(download_and_compile_csm)

#Install csm library (requires previous call to catkin_package)
function(install_csm)
    _set_csm_dirs()

    install(FILES "${CSM_BINARY_DIR}/install/lib/libcsm.so"
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

    install(DIRECTORY ${CSM_BINARY_DIR}/install/include/
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION} )

endfunction(install_csm)
