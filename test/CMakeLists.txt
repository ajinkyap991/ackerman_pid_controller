set(GTEST_SHUFFLE 1)

add_executable(
    cpp-test
    main.cpp
    mocktest.hpp
    gmockstatic.hpp
    test.cpp
    ../app/pidcontroller.cpp
    ../app/pidbase.cpp
)
target_include_directories(cpp-test PUBLIC ../vendor/googletest/googletest/include ${CMAKE_SOURCE_DIR}/include)
target_include_directories(cpp-test PUBLIC ../vendor/googletest/googlemock/include ${CMAKE_SOURCE_DIR}/include)

target_link_libraries(cpp-test PUBLIC gtest PUBLIC gmock)