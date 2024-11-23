// sample_test.cpp
#include <gtest/gtest.h>

// 被测函数
int add(int a, int b) {
    return a + b;
}

// 测试 add 函数
TEST(AdditionTest, PositiveNumbers) {
    EXPECT_EQ(add(3, 4), 7);
}

TEST(AdditionTest, NegativeNumbers) {
    EXPECT_EQ(add(-3, -4), -7);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
