# Try to find system libfort first
find_package(libfort CONFIG QUIET)

if(NOT libfort_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        libfort_proj
        QUIET
        GIT_REPOSITORY https://github.com/seleznevae/libfort.git
        GIT_TAG v0.5.1
    )
    FetchContent_MakeAvailable(libfort_proj)
    # Alias target for compatibility if needed
    if(NOT TARGET libfort::libfort AND TARGET libfort)
        add_library(libfort::libfort ALIAS libfort)
    endif()
endif()
