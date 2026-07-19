#pragma once
#include <string>
#include <gtest/gtest.h>
#include <sstream>

#define mplTest TEST_F
#define mplParameterizeTest TEST_P
#define mplInstantiateTestSuiteData INSTANTIATE_TEST_SUITE_P

// variadic macros
//https://learn.microsoft.com/en-us/cpp/preprocessor/variadic-macros?view=msvc-170

// generic formatter
template <typename... Args>
std::string mplFormat(Args&&... args) {
    std::ostringstream oss;
    (oss << ... << args);
    return oss.str();
}

// logging helpers
template <typename... Args>
inline void mplTestPrint(Args&&... args) {
    GTEST_LOG_(INFO) << mplFormat(std::forward<Args>(args)...);
}

template <typename... Args>
inline void mplTestError(Args&&... args) {
    GTEST_LOG_(ERROR) << mplFormat(std::forward<Args>(args)...);
}

template <typename... Args>
inline void mplTestWarning(Args&&... args) {
    GTEST_LOG_(WARNING) << mplFormat(std::forward<Args>(args)...);
}

class mplTestFixture : public ::testing::Test
{
    public:
        // Constructor
        mplTestFixture()
        {
        
        }
        // Destructor
        ~mplTestFixture()
        {

        }
    protected:
        // Setup for the test fixture.
        void SetUp() override
        {

        }
        // Clear for the test fixture.
        void TearDown() override
        {

        }
        std::string failureMessages;


};

template <typename T>
class mplTestParameterize : public mplTestFixture, public ::testing::WithParamInterface<T>
{
    public:
        mplTestParameterize() = default;
        ~mplTestParameterize() override = default;
    protected:
        void SetUp() override
        {
            mplTestFixture::SetUp();
        }

        void TearDown() override
        {
            mplTestFixture::TearDown();
        }
};