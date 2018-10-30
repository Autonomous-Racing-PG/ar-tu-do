// StdLib
#include <cstdlib>

// Own
// ...

// Testing
#include <gtest/gtest.h>

TEST(dummy_test, dummy_test_01)
{
    ASSERT_EQ(EXIT_SUCCESS, EXIT_SUCCESS);
}

TEST(dummy_test, dummy_test_02)
{
    ASSERT_EQ(1, 1);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
