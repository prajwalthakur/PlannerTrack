// Author Prajwal Thakur 
#include "project_utils/random_type.hpp"
#include <iostream>
namespace mpl::random_number
{
    std::string toString(const crRandomType& type)
    {
        switch(type)
        {
            case crRandomType::R_NONE:
                return "none";
            case crRandomType::R_UNIFORM_REAL:
                return "uniform_real";
            case crRandomType::R_NORMAL:
                return "normal";
            default:
                return "not None";
        }
    }

    crRandomType toRandomType(const std::string& type)
    {
        if(type == "none")
        {
            return crRandomType::R_NONE;

        }
        else if(type == "uniform_real")
        {
            return crRandomType::R_UNIFORM_REAL;
        }
        else if(type == "normal")
        {
            return crRandomType::R_NORMAL; 
        }
        else 
        {
            return crRandomType::R_NONE;
        }

    }
    std::unordered_set<std::string> getRandomTypes()
    {
        std::unordered_set<std::string> randomCollection;
        for (int i = 0; i < static_cast<int>(crRandomType::R_COUNT); ++i) 
        {
            auto name = toString(static_cast<crRandomType>(i));
            randomCollection.insert(name);    
        }
        return randomCollection;
    }

}//mpl::random_number
