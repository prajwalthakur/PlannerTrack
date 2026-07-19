
#pragma once
#include <iostream>
#include <optional>
#include <string>
#include "gtest/gtest.h"
#include "one_instance.hpp"
class mplTestManager : public crOnceInstance<mplTestManager>
{
    public:
        // Constructor.
        mplTestManager() = default;
        // Destructor.
        ~mplTestManager() = default;

    public:
        // numRuns - Sets the number of times to run this suite of tests.
        inline static int pseudoMain(int argc_, char** argv_, std::optional<int> numRun = std::nullopt)
        {
            instance().mNumRuns = numRun;
            return instance().runTests(argc_, argv_);
        }
        // Wrapper call to run the tests.
        inline int runTests(int argc_, char** argv_){
            testing::InitGoogleTest(&argc_, argv_);
            return RUN_ALL_TESTS();
        }

    private:
        // Number of runs to do in a test.
        std::optional<int> mNumRuns;
};

