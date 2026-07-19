// Author Prajwal Thakur 
#pragma once
#include <string>
#include "project_utils/common_utils.hpp"



enum  class EntityType
{
    AGENT=0,
    OBSTACLE=1,  //obstacles are those wehich are not controllable, 
    LANE=2, 
    NONE=3
};

enum class EntityShapeType
{
    CIRCLE=0,
    LINE_SEGMENT=1,
    CONVEX_POLYGON=2,
    NONE=3
};

inline EntityType getEntityType(const std::string& type)
{
    if (isStringEqual(type, "agent")) return EntityType::AGENT;
    if (isStringEqual(type, "obstacle")) return EntityType::OBSTACLE;
    if (isStringEqual(type, "lane")) return EntityType::LANE;
    return EntityType::NONE;
}

inline std::string toEntityString(EntityType type)
{
    switch (type)
    {
    case EntityType::AGENT: return "agent";
    case EntityType::OBSTACLE: return "obstacle";
    case EntityType::LANE: return "lane";
    default: return "none";
    }
}

inline EntityShapeType getEntityShapeType(const std::string& type)
{
    if (isStringEqual(type, "circle")) return EntityShapeType::CIRCLE;
    if (isStringEqual(type, "line_segment")) return EntityShapeType::LINE_SEGMENT;
    if (isStringEqual(type, "convex_polygon")) return EntityShapeType::CONVEX_POLYGON;
    return EntityShapeType::NONE;
}

inline std::string toShapeString(EntityShapeType type)
{
    switch (type)
    {
    case EntityShapeType::CIRCLE: return "circle";
    case EntityShapeType::LINE_SEGMENT: return "line_segment";
    case EntityShapeType::CONVEX_POLYGON: return "convex_polygon";
    default: return "none";
    }
}