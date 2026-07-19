// Author Prajwal Thakur 
#include "project_utils/common_utils.hpp"

void load_map(const std::string& map_path, std::vector<float>& values_buf, mt::MapArrayXfRow & mat_map_out)
{
    int numCOl =3;
    std::ifstream file(map_path); //map_path is  a string to the .csv file containing
    std::string line;
    //std::vector<float> values_vec;
    while(std::getline(file,line)){
        std::stringstream ss(line);
        std::string word; 
        while(std::getline(ss,word,','))
        {
            values_buf.push_back(std::stof(word)); // creating a row of 3x1
        }
    }
    int rows = values_buf.size()/numCOl;
    new (&mat_map_out) mt::MapArrayXfRow(values_buf.data(), rows, numCOl);
    return;
}

std::string str_tolower(std::string s)
{
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c){ return std::tolower(c); } 
                  );
    return s;
}

////////////////////////////////////////////////////////////////////////////////

bool isStringEqual(const std::string& s1, const std::string& s2)
{
    std::string s1temp = str_tolower(s1);
    std::string s2temp = str_tolower(s2);
    if(s1temp.compare(s2temp)==0)
        return true;
    else
        return false;
}


std::string concatString(std::vector<std::string>& vec)
{
    std::string newString;
    for(auto& str : vec)
    {
        newString =  newString.append(str);
    }
    return newString;
}

double deg2rad(double degree)
{
    double radian = degree*0.0174533;
    return radian;
}

std::string intToString(int i)
{
    std::string res;
    res = std::to_string(i);
    return res;
}