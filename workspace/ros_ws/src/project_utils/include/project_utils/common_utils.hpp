// Author Prajwal Thakur 
#pragma once
#include "project_utils/types.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
void load_map(const std::string& map_path, std::vector<float>& values_buf, mt::MapArrayXfRow & mat_map_out);

////////////////////////////////////////////////////////////////////////////////

std::string str_tolower(std::string s);

////////////////////////////////////////////////////////////////////////////////

bool isStringEqual(const std::string& s1, const std::string& s2);

////////////////////////////////////////////////////////////////////////////////

std::string concatString(std::vector<std::string>& vec);

////////////////////////////////////////////////////////////////////////////////

double deg2rad(double degree);

std::string intToString(int i);


