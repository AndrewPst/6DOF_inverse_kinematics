
#include "Matrix.h"
#include "Kinematics.h"

#include <array>
#include <iostream>

void printPos(const Position_t& pos, std::ostream &stream)
{
    stream << "===Position===\n";
    stream << "X = " << pos.x << '\n';
    stream << "Y = " << pos.y << '\n';
    stream << "Z = " << pos.z << '\n';
    stream << "Wx = " << pos.wx << '\n';
    stream << "Wy = " << pos.wy << '\n';
    stream << "Wz = " << pos.wz << '\n';
    stream << "==============\n";
}

void printAngles(const std::vector<double> ang, std::ostream& stream, bool convert_to_degrees = true)
{
    double koef = convert_to_degrees ? 180.0 / M_PI : 1;
    stream << "====Angles====\n";
    for(size_t i = 0; i < ang.size(); i++)
        stream << "№" << i << '\t' << ang[i] * koef << std::endl;
    stream << "==============\n";
}

int main()
{
    //Setup manipulator parameters
    Manipulator_t<6> man;
    man.alfa = {-M_PI_2, 0, -M_PI_2, M_PI_2, -M_PI_2, 0};
    man.theta = {0, -M_PI_2, 0, 0, 0, 0};
    man.r = {0, 110, 0, 0, 0, 0};
    man.d = {133, 0, 0, 117.5, 0, 28};

    //init kinematics calculator
    KinematicsCalc kin(std::move(man));
    Position_t pos;

    kin.forwardKinematicsOptimized({-M_PI_4, 0, 0, M_PI_4, M_PI_4, 0}, pos);
    printPos(pos, std::cout);

    std::vector<double> out(6);
    kin.inverseKinematicsOptimized(pos, out);

    printAngles(out, std::cout, true);

    return 0;
}