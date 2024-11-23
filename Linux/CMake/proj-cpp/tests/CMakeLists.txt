
set(GTEST_LIB_DIR ${CMAKE_SOURCE_DIR}/libs)
set(GTEST_MAIN_LIB_DIR ${CMAKE_SOURCE_DIR}/libs)

add_executable(sample_test sample_test.cpp)

target_link_libraries(sample_test PUBLIC
    ${GTEST_LIB_DIR}/libgtest.so
    ${GTEST_MAIN_LIB_DIR}/libgtest_main.so
    pthread
)