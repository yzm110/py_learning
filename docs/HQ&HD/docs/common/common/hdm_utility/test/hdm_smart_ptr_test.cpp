#include "gtest/gtest.h"
#include "hdm_utility/hdm_smart_ptr.h"

using namespace hdm_utility;

class CHdmSmartPtrTest : public ::testing::Test
{
    public:


    virtual void SetUp() 
    { 

    }

    virtual void TearDown() 
    { 

    }
};


TEST_F(CHdmSmartPtrTest, Wgs84ToGcj02)
{
    CHdmSmartPtr<int> my_int1 = CHdmSmartPtr<int>(new int(5));
    printf("my_int1=[%d]\n", *my_int1);
    CHdmSmartPtr<int> my_int2 = my_int1;
    printf("my_int2=[%d]\n", *my_int2);
    CHdmSmartPtr<int> my_int3 = CHdmSmartPtr<int>(new int(6));
    printf("my_int3=[%d]\n", *my_int3);
    my_int1 = my_int3;
    printf("my_int1=[%d], my_int2=[%d], my_int3=[%d]\n", *my_int1, *my_int2, *my_int3);
}


