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
    GIT_TAG e8c379203792fcad764cce239986b325619475fd
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
FetchContent_Declare(
    mathoptsolverscmake
    GIT_REPOSITORY https://github.com/fontanf/mathoptsolverscmake.git
    GIT_TAG 7a2b2078fe6dba5f26b6ea46e525e3fe817b429a
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../mathoptsolverscmake/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(mathoptsolverscmake)

set(KNAPSACKSOLVER_BUILD_MAIN OFF)
set(KNAPSACKSOLVER_BUILD_TEST OFF)
FetchContent_Declare(
    knapsacksolver
    GIT_REPOSITORY https://github.com/fontanf/knapsacksolver.git
    GIT_TAG 1d7a689f88e953c099087a31f174cb6646002f9f
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../knapsacksolver/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(knapsacksolver)

set(MULTIPLECHOICEKNAPSACKSOLVER_BUILD_MAIN OFF)
set(MULTIPLECHOICEKNAPSACKSOLVER_BUILD_TEST OFF)
if(KNAPSACKWITHCONFLICTSSOLVER_USE_HIGHS)
    set(MULTIPLECHOICEKNAPSACKSOLVER_USE_HIGHS ON)
endif()
FetchContent_Declare(
    multiplechoiceknapsacksolver
    GIT_REPOSITORY https://github.com/fontanf/multiplechoiceknapsacksolver.git
    GIT_TAG 98cddecafa6c2af03481747b885a053eb42f534b
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../multiplechoiceknapsacksolver/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(multiplechoiceknapsacksolver)

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
    GIT_TAG 556971624c92c23c961371ce5721ff9eb920a874)
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../columngenerationsolver/")
FetchContent_MakeAvailable(columngenerationsolver)

# Fetch fontanf/stablesolver.
set(STABLESOLVER_BUILD_MAIN OFF)
set(STABLESOLVER_BUILD_TEST OFF)
if(KNAPSACKWITHCONFLICTSSOLVER_USE_CBC)
    set(STABLESOLVER_USE_CBC ON)
endif()
FetchContent_Declare(
    stablesolver
    GIT_REPOSITORY https://github.com/fontanf/stablesolver.git
    GIT_TAG 673f47a1daa32d5eb33a09083bb15a87b7c2fb65
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
    GIT_TAG c7e7e658fdbb6c43c3218c4c11417e9afb7691ae
    #SOURCE_DIR "${PROJECT_SOURCE_DIR}/../setcoveringsolver/"
    EXCLUDE_FROM_ALL)
FetchContent_MakeAvailable(setcoveringsolver)
