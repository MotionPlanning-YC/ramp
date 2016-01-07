#include <gtest/gtest.h>

// bad function:
// for example: how to deal with overflow?
int add(int a, int b){
    return a + b;
}

TEST(NumberCmpTest, ShouldPass1){
    ASSERT_EQ(3, add(1,2));
}

TEST(NumberCmpTest, ShouldPass2){
    EXPECT_TRUE(4 != add(1,2));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
