find_program(CTEST_GIT_COMMAND NAMES git)

set(CTEST_BUILD_CONFIGURATION "Release")
set(CTEST_CONFIGURATION_TYPE "Release")

set(CTEST_BUILD_OPTIONS "-DBUILD_allproducer=ON -D ReferenceDataDir=${ReferenceDataDir}")

# valgrind is used for memchecks
find_program(CTEST_MEMORYCHECK_COMMAND NAMES valgrind)

# remove python related valgrind messages
set(CTEST_MEMORYCHECK_SUPPRESSIONS_FILE ${CTEST_SOURCE_DIRECTORY}/etc/tests/valgrind/valgrind-python.supp)

# construct the CMake command
set(CTEST_CONFIGURE_COMMAND "${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE:STRING=${CTEST_BUILD_CONFIGURATION}")
set(CTEST_CONFIGURE_COMMAND "${CTEST_CONFIGURE_COMMAND} ${CTEST_BUILD_OPTIONS}")
set(CTEST_CONFIGURE_COMMAND "${CTEST_CONFIGURE_COMMAND} \"-G${CTEST_CMAKE_GENERATOR}\"")
set(CTEST_CONFIGURE_COMMAND "${CTEST_CONFIGURE_COMMAND} \"${CTEST_SOURCE_DIRECTORY}\"")

ctest_start("Nightly")

# clear binary dir before starting (build directory)
ctest_empty_binary_directory(${CTEST_BINARY_DIRECTORY})

ctest_update()
ctest_configure()
ctest_build(TARGET install)
ctest_test()
if (WITH_MEMCHECK)
  ctest_memcheck()
endif (WITH_MEMCHECK)
ctest_submit()
