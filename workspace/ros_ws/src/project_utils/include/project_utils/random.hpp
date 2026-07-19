/**
 * @file Random.hpp
 * @author Prajwal Thakur (you@domain.com)
 * @brief  A class for generating scalar random numbers 
 * ref: https://github.com/prajwalthakur/CppRoboPlan-Public/blob/release/workspace/core/Random.hpp
 * */


#pragma once 
#include "project_utils/random_type.hpp"
#include  <optional>
#include <random>
#include <limits>

namespace mpl::random_number
{

    class crRandomGenerator
    {
    public:
        crRandomGenerator();
        crRandomGenerator(const std::size_t seed);
        ~crRandomGenerator()=default;

        void reset();

        void setSeed(const std::size_t seedValue);

        void setRange(
            double min,
            double max,
            std::optional<double> mean = std::nullopt,
            std::optional<double> std_dev = std::nullopt);

        bool setDistribution(const std::string& str);

        double getRandomNumber();

    private:
        bool randNumberNone();
        bool randNumberUniform();
        bool randNumberNormal();

    private:
        std::size_t mSeed;

        double mMin = -std::numeric_limits<double>::infinity();
        double mMax =  std::numeric_limits<double>::infinity();

        crRandomType mType = crRandomType::R_NONE;

        std::optional<double> mMean = std::nullopt;
        std::optional<double> mStdDev = std::nullopt;

        std::mt19937 mDefaultGenerator;

        std::uniform_real_distribution<double> mUniformReal;
        std::normal_distribution<double> mNormalReal;
    };

} // namespace mpl::random_number
