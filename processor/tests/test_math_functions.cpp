#include <gtest/gtest.h>
#include "math_functions.h"

// Test case for the add function
TEST(AddFunctionTest, PositiveNumbers) {
    EXPECT_EQ(add(1, 2), 3);
    EXPECT_EQ(add(10, 20), 30);
}

TEST(AddFunctionTest, NegativeNumbers) {
    EXPECT_EQ(add(-1, -2), -3);
    EXPECT_EQ(add(-10, -20), -30);
}

TEST(AddFunctionTest, MixedNumbers) {
    EXPECT_EQ(add(-1, 1), 0);
    EXPECT_EQ(add(10, -5), 5);
}