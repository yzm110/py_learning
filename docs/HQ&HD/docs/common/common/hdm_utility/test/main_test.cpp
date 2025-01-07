
#include "gtest/gtest.h"

class CEhrAlgoEnviroment : public testing::Environment
{
    virtual void SetUp() 
    { 

    }

    virtual void TearDown() 
    { 

    }
};

int32_t main(int32_t argc, char** argv)
{
    testing::AddGlobalTestEnvironment(new CEhrAlgoEnviroment());
    testing::InitGoogleTest(&argc, argv);

    return RUN_ALL_TESTS();
}

