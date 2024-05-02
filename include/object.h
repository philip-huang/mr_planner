#ifndef MR_PLANNER_OBJECT_H
#define MR_PLANNER_OBJECT_H

#include <string>
#include <memory>
#include <vector>

struct Object  {
    enum State {
        Static = 0,
        Attached = 1,
        Supported = 2,
    };
    enum Shape {
        Box = 0,
        Sphere = 1,
        Cylinder = 2,
        Mesh = 3,
    };

    std::string name;
    // mode of the object
    State state;
    std::string parent_link; 
    int robot_id;

    // geometry of the object
    double x;
    double y;
    double z;
    double qx;
    double qy;
    double qz;
    double qw;

    // collision shape of the object
    Shape shape;
    double radius;
    double length; // x
    double width; // y
    double height; // z
    std::string mesh_path;
};

#endif //MR_PLANNER_OBJECT_H