//
// Created by Mahi on 6/12/20.
//

#ifndef GYMSIM_ROBOTSFORMATION_H
#define GYMSIM_ROBOTSFORMATION_H

#include "common.hpp"
#include "configwidget.h"
#include "robot.h"
#include <QFile>
#include <QTextStream>

class RobotsFormation {
public:
    float x[MAX_ROBOT_COUNT]{};
    float y[MAX_ROBOT_COUNT]{};
    RobotsFormation(int type, ConfigWidget* _cfg);
    void setAll(double *xx,double *yy);
//    void loadFromFile(const QString& filename);
    void resetRobots(SimRobot** r,int team);
private:
    ConfigWidget* cfg;
};

#endif //GYMSIM_ROBOTSFORMATION_H
