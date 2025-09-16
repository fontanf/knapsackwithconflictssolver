# Enable FetchContent.
include(FetchContent)

# Fetch boost.
if(KNAPSACKWITHCONFLICTSSOLVER_BUILD_MAIN)
    set(BOOST_INCLUDE_LIBRARIES thread filesystem system program_options dynamic_bitset)
    set(BOOST_ENABLE_CMAKE ON)
    include(FetchContent)
    FetchContent_Declare(
        Boost
        URL https://github.com/boostorg/boost/releases/download/boost-1.84.0/boost-1.84.0.tar.xz
        EXCLUDE_FROM_ALL)
    FetchContent_MakeAvailable(Boost)
endif()

# Fetch fontanf/optimizationtools.
FetchContent_Declare(
    optimizationtools
    GIT_REPOSITORY https://github.com/fontanf/optimizationtools.git
    GIT_TAG 6d2411a74230be157d1ede47e88d30a68854b8ac
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
set(MATHOPTSOLVERSCMAKE_USE_HIGHS ON)
FetchContent_Declare(
    mathoptsolverscmake
    GIT_REPOSITORY https://github.com/fontanf/mathoptsolverscmake.git
    GIT_TAG 59fd908038906df649e121f42652a0e8f99cbc67
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../mathoptsolverscmake/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(mathoptsolverscmake)

# Fetch fontanf/treesearchsolver.
set(TREESEARCHSOLVER_BUILD_TEST OFF)
set(TREESEARCHSOLVER_BUILD_EXAMPLES OFF)
FetchContent_Declare(
    treesearchsolver
    GIT_REPOSITORY https://github.com/fontanf/treesearchsolver.git
    GIT_TAG 8cf4a179d103efb9d4e0a94bc2f6f669105cd45a)
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../treesearchsolver/")
FetchContent_MakeAvailable(treesearchsolver)

# Fetch fontanf/localsearchsolver.
set(LOCALSEARCHSOLVER_BUILD_TEST OFF)
set(LOCALSEARCHSOLVER_BUILD_EXAMPLES OFF)
FetchContent_Declare(
    localsearchsolver
    GIT_REPOSITORY https://github.com/fontanf/localsearchsolver.git
    GIT_TAG 7a35a7d3017324ebc0b74d5de8d4b001b19fcbed)
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../localsearchsolver/")
FetchContent_MakeAvailable(localsearchsolver)

set(STABLESOLVER_USE_CBC OFF)
FetchContent_Declare(
    stablesolver
    GIT_REPOSITORY https://github.com/fontanf/stablesolver.git
    GIT_TAG 3c374c789c766111ac953e2a46d21881f5841cd0
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../stablesolver/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(stablesolver)

FetchContent_Declare(
    knapsacksolver
    GIT_REPOSITORY https://github.com/fontanf/knapsacksolver.git
    GIT_TAG 2b2bead64fc7b84ab57c23d3f906e0526cfa71a0
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../knapsacksolver/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(knapsacksolver)
