#ifndef TOOL_H_
#define TOOL_H_
namespace  costmap_converter
{
    struct costmapinfo
    {
        int xcellSize,yCellSize;
        double resolution;
        double xMetersSize,yMetersSize;
        double orgin_x,orgin_y;
    };
}

#endif