#include "UniqueId.h"

////////////////////////////////////////////////////////////////////////////////

UniqueId::UniqueId(const std::string& category, long int value)
    :mValue(value)
{
    // if value ==-1, we are creating the instance for first time
    if(value == -1)
    {
        mValue = mGen.getUniqueValue(category);
        setCategoryValue(mCategory,mValue);
    }
    // otherwise it already exists in the server, and we are creating the clone 
    // to work with the algorithms
    mCategory = category;
    mValueStr = std::to_string(mValue);
    setCategoryValue();
}

////////////////////////////////////////////////////////////////////////////////

void UniqueId::setCategoryValue(const std::string& category, long int value)
{
    if(value<=-1)
        std::cerr<<"UniqueId: value should be greater than -1";
    mGen.setCategoryValue(category,value);
    
}

////////////////////////////////////////////////////////////////////////////////

long int UniqueId::value() const
{
    return mValue;
}

////////////////////////////////////////////////////////////////////////////////

const std::string& UniqueId::category() const
{
    return mCategory;
}

////////////////////////////////////////////////////////////////////////////////

void UniqueId::setCategoryValue()
{
    mValueCategoryStr = mValueStr + mCategory;
}

////////////////////////////////////////////////////////////////////////////////

void UniqueId::print() const
{
    if (mCategory.empty())
        fmt::print(
            "Category : (null) {:<12} Value : {:<5}\n",
            "",
            mValue
        );
    else
        fmt::print(
            "Category : {:<19} Value : {:<5}\n",
            mCategory,
            mValue
        );
}

////////////////////////////////////////////////////////////////////////////////

bool UniqueId::operator==(const UniqueId& other) const
{
    return mValueCategoryStr == other.mValueCategoryStr;
}