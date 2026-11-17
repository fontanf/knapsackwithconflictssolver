# Enable FetchContent.
include(FetchContent)

# Fetch boost.
# SMS++ requires Boost, even if the main executable isn't built.
if(KNAPSACKWITHCONFLICTSSOLVER_BUILD_MAIN OR KNAPSACKWITHCONFLICTSSOLVER_USE_SMSPP)
    set(BOOST_INCLUDE_LIBRARIES thread filesystem system program_options dynamic_bitset)
    if(KNAPSACKWITHCONFLICTSSOLVER_USE_SMSPP)
        # Header-only Boost libraries #include'd directly by SMS++/BundleSolver
        # (e.g. SMS++'s AbstractBlock.cpp includes <boost/algorithm/string.hpp>).
        # Boost's modular CMake build only exposes headers for libraries listed
        # here, even though the boost-*.tar.xz release ships all of Boost's
        # sources.
        list(APPEND BOOST_INCLUDE_LIBRARIES algorithm any bimap function functional multi_array preprocessor)
    endif()
    set(BOOST_ENABLE_CMAKE ON)
    include(FetchContent)
    FetchContent_Declare(
        Boost
        URL https://github.com/boostorg/boost/releases/download/boost-1.84.0/boost-1.84.0.tar.xz
        EXCLUDE_FROM_ALL
        OVERRIDE_FIND_PACKAGE)
    FetchContent_MakeAvailable(Boost)

    # Boost fetched via FetchContent only defines per-component targets (e.g.
    # Boost::filesystem), and Boost::headers is just an (essentially empty) meta
    # package, NOT an aggregate of every other component's headers -- unlike a
    # classic system-wide Boost install, where one include path exposes all of
    # Boost regardless of which libraries are actually used. The "Boost::boost"
    # compatibility target normally provided by a plain find_package(Boost) is
    # not created either, so define it here, linked against every library we
    # fetched, so dependencies (e.g. SMS++) that link against Boost::boost
    # directly and #include whichever Boost headers they need can still find
    # them all.
    if(NOT TARGET Boost::boost)
        add_library(Boost::boost INTERFACE IMPORTED)
        foreach(_boost_lib IN LISTS BOOST_INCLUDE_LIBRARIES)
            target_link_libraries(Boost::boost INTERFACE Boost::${_boost_lib})
        endforeach()
    endif()
endif()

# Fetch fontanf/optimizationtools.
FetchContent_Declare(
    optimizationtools
    GIT_REPOSITORY https://github.com/fontanf/optimizationtools.git
    GIT_TAG cf0e77052f300ca90eb533d59526955e6f8c8858
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../optimizationtools/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(optimizationtools)

# Fetch fontanf/mathoptsolverscmake.
if(KNAPSACKWITHCONFLICTSSOLVER_USE_CBC)
    set(MATHOPTSOLVERSCMAKE_USE_CBC ON)
endif()
if(KNAPSACKWITHCONFLICTSSOLVER_USE_XPRESS)
    set(MATHOPTSOLVERSCMAKE_USE_XPRESS ON)
endif()
if(KNAPSACKWITHCONFLICTSSOLVER_USE_HIGHS)
    set(MATHOPTSOLVERSCMAKE_USE_HIGHS ON)
endif()
if(KNAPSACKWITHCONFLICTSSOLVER_USE_KNITRO)
    set(MATHOPTSOLVERSCMAKE_USE_KNITRO ON)
endif()
if(KNAPSACKWITHCONFLICTSSOLVER_USE_DLIB)
    set(MATHOPTSOLVERSCMAKE_USE_DLIB ON)
endif()
if(KNAPSACKWITHCONFLICTSSOLVER_USE_CONICBUNDLE)
    set(MATHOPTSOLVERSCMAKE_USE_CONICBUNDLE ON)
endif()
if(KNAPSACKWITHCONFLICTSSOLVER_USE_SMSPP)
    set(MATHOPTSOLVERSCMAKE_USE_SMSPP ON)
endif()
FetchContent_Declare(
    mathoptsolverscmake
    GIT_REPOSITORY https://github.com/fontanf/mathoptsolverscmake.git
    GIT_TAG 9dfd26a4cb4c58cb30045af22bda667b303887c3
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../mathoptsolverscmake/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(mathoptsolverscmake)

# Fetch fontanf/treesearchsolver.
set(TREESEARCHSOLVER_BUILD_TEST OFF)
set(TREESEARCHSOLVER_BUILD_EXAMPLES OFF)
FetchContent_Declare(
    treesearchsolver
    GIT_REPOSITORY https://github.com/fontanf/treesearchsolver.git
    GIT_TAG 4795f547e8725817b11472ced5b393c3a5d46577)
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../treesearchsolver/")
FetchContent_MakeAvailable(treesearchsolver)

# Fetch fontanf/localsearchsolver.
set(LOCALSEARCHSOLVER_BUILD_TEST OFF)
set(LOCALSEARCHSOLVER_BUILD_EXAMPLES OFF)
FetchContent_Declare(
    localsearchsolver
    GIT_REPOSITORY https://github.com/fontanf/localsearchsolver.git
    GIT_TAG 6cc833ea92eaf8bcac0e5e0c6119a0c936d58457)
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../localsearchsolver/")
FetchContent_MakeAvailable(localsearchsolver)

# Fetch fontanf/columngenerationsolver.
set(COLUMNGENERATIONSOLVER_BUILD_TEST OFF)
set(COLUMNGENERATIONSOLVER_BUILD_EXAMPLES OFF)
if(KNAPSACKWITHCONFLICTSSOLVER_USE_CLP)
    set(COLUMNGENERATIONSOLVER_USE_CLP ON)
endif()
if(KNAPSACKWITHCONFLICTSSOLVER_USE_XPRESS)
    set(COLUMNGENERATIONSOLVER_USE_XPRESS ON)
endif()
if(KNAPSACKWITHCONFLICTSSOLVER_USE_HIGHS)
    set(COLUMNGENERATIONSOLVER_USE_HIGHS ON)
endif()
FetchContent_Declare(
    columngenerationsolver
    GIT_REPOSITORY https://github.com/fontanf/columngenerationsolver.git
    GIT_TAG 73b9332651f59a177cf633ad46528a630aa982c2)
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../columngenerationsolver/")
FetchContent_MakeAvailable(columngenerationsolver)

# Fetch fontanf/knapsacksolver.
set(KNAPSACKSOLVER_BUILD_MAIN OFF)
set(KNAPSACKSOLVER_BUILD_TEST OFF)
FetchContent_Declare(
    knapsacksolver
    GIT_REPOSITORY https://github.com/fontanf/knapsacksolver.git
    GIT_TAG 1d7a689f88e953c099087a31f174cb6646002f9f
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../knapsacksolver/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(knapsacksolver)

# Fetch fontanf/multiplechoiceknapsacksolver.
set(MULTIPLECHOICEKNAPSACKSOLVER_BUILD_MAIN OFF)
set(MULTIPLECHOICEKNAPSACKSOLVER_BUILD_TEST OFF)
if(KNAPSACKWITHCONFLICTSSOLVER_USE_HIGHS)
    set(MULTIPLECHOICEKNAPSACKSOLVER_USE_HIGHS ON)
endif()
FetchContent_Declare(
    multiplechoiceknapsacksolver
    GIT_REPOSITORY https://github.com/fontanf/multiplechoiceknapsacksolver.git
    GIT_TAG 041c7c217e2fdd4c92ffbaab384e42b585ed5db6
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../multiplechoiceknapsacksolver/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(multiplechoiceknapsacksolver)

# Fetch fontanf/stablesolver.
set(STABLESOLVER_BUILD_MAIN OFF)
set(STABLESOLVER_BUILD_TEST OFF)
if(KNAPSACKWITHCONFLICTSSOLVER_USE_CBC)
    set(STABLESOLVER_USE_CBC ON)
endif()
FetchContent_Declare(
    stablesolver
    GIT_REPOSITORY https://github.com/fontanf/stablesolver.git
    GIT_TAG eeb6147adbf1c24ed1131f68ca266a41d41883b9
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../stablesolver/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(stablesolver)

set(SETCOVERINGSOLVER_BUILD_MAIN OFF)
set(SETCOVERINGSOLVER_BUILD_TEST OFF)
if(KNAPSACKWITHCONFLICTSSOLVER_USE_CLP)
    set(SETCOVERINGSOLVER_USE_CLP ON)
endif()
if(KNAPSACKWITHCONFLICTSSOLVER_USE_CBC)
    set(SETCOVERINGSOLVER_USE_CBC ON)
endif()
FetchContent_Declare(
    setcoveringsolver
    GIT_REPOSITORY https://github.com/fontanf/setcoveringsolver.git
    GIT_TAG 1992d69c16a136e00685581ccc996e140e231f6a
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../setcoveringsolver/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(setcoveringsolver)
