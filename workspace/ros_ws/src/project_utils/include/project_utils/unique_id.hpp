// Author Prajwal Thakur 
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <iostream>
#include <string>
#include <tuple>
#include <functional>
#include <fmt/core.h>

#include "project_utils/unique_id_generator.hpp"
#include "project_utils/env_entity_type.hpp"
#include "project_utils/common_utils.hpp"

class UniqueId
{
public:
    UniqueId() = default;
    ~UniqueId() = default;

    UniqueId(const std::string& category, long int value = -1);
    // UniqueId(EntityType entType, long int value = -1);

    static void setCategoryValue(const std::string& category,
                                 long int value = -1);

    // Getters
    long int value() const;
    const std::string& category() const;

    void print() const;

    // Comparison operators

    // Required for unordered_map / unordered_set
    bool operator==(const UniqueId& other) const
    {
        return (mCategory == other.mCategory) &&
               (mValue == other.mValue);
    }

    // Required for map / set
    bool operator<(const UniqueId& other) const
    {
        return std::tie(mCategory, mValue) <
               std::tie(other.mCategory, other.mValue);
    }
    inline std::string getCategoryValue() const {return mValueCategoryStr;}
private:
    void setCategoryValue();

private:
    // Unique numeric value
    long int mValue{-1};

    // String representation of value
    std::string mValueStr{"-1"};

    // Category name
    std::string mCategory{"-1"};

    // Combined representation: category + value
    std::string mValueCategoryStr{"-1"};

    // Static generator shared by all UniqueId objects
    static inline UniqueIdGenerator mGen;
};


// -----------------------------------------------------------------------------
// Hash helper (do NOT place inside namespace std)
// -----------------------------------------------------------------------------

namespace detail
{
    inline std::size_t combine_hash(std::size_t seed,
                                    std::size_t hash_value)
    {
        seed ^= hash_value + 0x9e3779b9 +
                (seed << 6U) +
                (seed >> 2U);

        return seed;
    }
}


// -----------------------------------------------------------------------------
// std::hash specialization (allowed)
// Required for unordered_map / unordered_set
// -----------------------------------------------------------------------------

namespace std
{
    template <>
    struct hash<UniqueId>
    {
        std::size_t operator()(const UniqueId& id) const noexcept
        {
            std::size_t h1 =
                std::hash<long int>{}(id.value());

            std::size_t h2 =
                std::hash<std::string>{}(id.category());

            return detail::combine_hash(h1, h2);
        }
    };
}