// Author Prajwal Thakur 
#include <iostream>
#include <optional>
#include <iostream>
#include <algorithm>
#include <ctime>
#include "project_utils/random.hpp"

namespace mpl::random_number
{
    
    crRandomGenerator::crRandomGenerator()
    {
        setSeed(time(0));
    }


    crRandomGenerator::crRandomGenerator(const size_t seed)
    {
        setSeed(seed);
    }

   
    void crRandomGenerator::setSeed(const std::size_t seed)
    {
        mSeed = seed;
        mDefaultGenerator.seed(mSeed);
    }

    
    bool crRandomGenerator::setDistribution(const std::string& type_str)
    {
        crRandomType type = toRandomType(type_str);

        switch(type)
        {
            case crRandomType::R_NONE:
            {
                mType = crRandomType::R_NONE;
                return randNumberNone();
            }

            case crRandomType::R_UNIFORM_REAL:
            {
                mType = crRandomType::R_UNIFORM_REAL;
                return randNumberUniform();
            }

            case crRandomType::R_NORMAL:
            {
                mType = crRandomType::R_NORMAL;
                return randNumberNormal();
            }

            default:
            {
                std::cerr << "Invalid distribution selection\n";
                return false;
            }
        }
    }

    
    void crRandomGenerator::setRange(
        double min,
        double max,
        std::optional<double> mean,
        std::optional<double> std_dev)
    {
        mMin = min;
        mMax = max;
        if(mean.has_value())
            mMean = mean;
        else
            mMean = (mMin + mMax) / 2.0;

        if(std_dev.has_value())
            mStdDev = std_dev;
        else
            mStdDev = (mMax - mMin) / 6.0;
    }

    
    bool crRandomGenerator::randNumberUniform()
    {
        mUniformReal.param(
            typename std::uniform_real_distribution<double>::param_type(
                mMin, mMax));

        return true;
    }

    
    bool crRandomGenerator::randNumberNormal()
    {
        if(!mMean.has_value() || !mStdDev.has_value())
        {
            std::cerr << "Mean or std deviation not set\n";
            return false;
        }
        mNormalReal.param(
            typename std::normal_distribution<double>::param_type(
                *mMean,
                *mStdDev));
        std::cerr<<"normal noise set";
        return true;
    }

    
    bool crRandomGenerator::randNumberNone()
    {
        return true;
    }

    
    double crRandomGenerator::getRandomNumber()
    {
        switch(mType)
        {
            case crRandomType::R_UNIFORM_REAL:
                return mUniformReal(mDefaultGenerator);

            case crRandomType::R_NORMAL:
            {
                double value;
                do
                {
                    value = mNormalReal(mDefaultGenerator);
                }
                while(value < mMin || value > mMax);

                return value;
            }

            case crRandomType::R_NONE:
                return mMean.value_or(0.0);

            default:
                return std::numeric_limits<double>::infinity();
        }
    }

    
    void crRandomGenerator::reset()
    {
        mDefaultGenerator.seed(mSeed);
    }

} // namespace mpl::random_number