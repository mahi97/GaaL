//
// Created by Mahi on 6/12/20.
//

#include "robotsformation.h"

void RobotsFormation::setAll(dReal *xx, dReal *yy) {
    for (int i = 0; i < MAX_ROBOT_COUNT; i++) {
        x[i] = xx[i];
        y[i] = yy[i];
    }
}

RobotsFormation::RobotsFormation(int
                                 type, ConfigWidget *_cfg) : cfg(_cfg) {
    if (type == 0) {
        dReal teamPosX[MAX_ROBOT_COUNT] = {2.20, 1.00, 1.00, 1.00, 0.33, 1.22,
                                           3.00, 3.20, 3.40, 3.60, 3.80, 4.00,
                                           0.40, 0.80, 1.20, 1.60};
        dReal teamPosY[MAX_ROBOT_COUNT] = {0.00, -0.75, 0.00, 0.75, 0.25, 0.00,
                                           1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
                                           -3.50, -3.50, -3.50, -3.50};
        setAll(teamPosX, teamPosY);
    }
    if (type == 1) // formation 1
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {1.50, 1.50, 1.50, 0.55, 2.50, 3.60,
                                           3.20, 3.20, 3.20, 3.20, 3.20, 3.20,
                                           0.40, 0.80, 1.20, 1.60};
        dReal teamPosY[MAX_ROBOT_COUNT] = {1.12, 0.0, -1.12, 0.00, 0.00, 0.00,
                                           0.75, -0.75, 1.50, -1.50, 2.25, -2.25,
                                           -3.50, -3.50, -3.50, -3.50};
        setAll(teamPosX, teamPosY);
    }
    if (type == 2) // formation 2
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {4.20, 3.40, 3.40, 0.70, 0.70, 0.70,
                                           2.00, 2.00, 2.00, 2.00, 2.00, 2.00,
                                           0.40, 0.80, 1.20, 1.60};
        dReal teamPosY[MAX_ROBOT_COUNT] = {0.00, -0.20, 0.20, 0.00, 2.25, -2.25,
                                           0.75, -0.75, 1.50, -1.50, 2.25, -2.25,
                                           -3.50, -3.50, -3.50, -3.50};
        setAll(teamPosX, teamPosY);
    }
    if (type == 3) // outside field
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {0.40, 0.80, 1.20, 1.60, 2.00, 2.40,
                                           2.80, 3.20, 3.60, 4.00, 4.40, 4.80,
                                           0.40, 0.80, 1.20, 1.60};
        dReal teamPosY[MAX_ROBOT_COUNT] = {-4.00, -4.00, -4.00, -4.00, -4.00, -4.00,
                                           -4.00, -4.00, -4.00, -4.00, -4.00, -4.00,
                                           -4.40, -4.40, -4.40, -4.40};
        setAll(teamPosX, teamPosY);
    }
    if (type == 4) {
        dReal teamPosX[MAX_ROBOT_COUNT] = {2.80, 2.50, 2.50, 0.80, 0.80, 1.10,
                                           3.00, 3.20, 3.40, 3.60, 3.80, 4.00,
                                           0.40, 0.80, 1.20, 1.60};
        dReal teamPosY[MAX_ROBOT_COUNT] = {5.00, 4.70, 5.30, 5.00, 6.50, 5.50,
                                           1.00, 1.00, 1.00, 1.00, 1.00, 1.00,
                                           -3.50, -3.50, -3.50, -3.50};
        setAll(teamPosX, teamPosY);
    }
    if (type == -1) // outside
    {
        dReal teamPosX[MAX_ROBOT_COUNT] = {0.40, 0.80, 1.20, 1.60, 2.00, 2.40,
                                           2.80, 3.20, 3.60, 4.00, 4.40, 4.80,
                                           0.40, 0.80, 1.20, 1.60};
        dReal teamPosY[MAX_ROBOT_COUNT] = {-3.40, -3.40, -3.40, -3.40, -3.40, -3.40,
                                           -3.40, -3.40, -3.40, -3.40, -3.40, -3.40,
                                           -3.20, -3.20, -3.20, -3.20};
        setAll(teamPosX, teamPosY);
    }

}

//void RobotsFormation::loadFromFile(const QString &filename) {
//    QFile file(filename);
//    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
//        return;
//    QTextStream in(&file);
//    int k;
//    for (k = 0; k < cfg->Robots_Count(); k++) x[k] = y[k] = 0;
//    k = 0;
//    while (!in.atEnd()) {
//        QString line = in.readLine();
//        QStringList list = line.split(",");
//        if (list.count() >= 2) {
//            x[k] = list[0].toFloat();
//            y[k] = list[1].toFloat();
//        } else if (list.count() == 1) {
//            x[k] = list[0].toFloat();
//        }
//        if (k == cfg->Robots_Count() - 1) break;
//        k++;
//    }
//}

void RobotsFormation::resetRobots(SimRobot **r, int team) {
    dReal dir = -1;
    int cnt = cfg->Yellow_Robots_Count();
    if (team == 1) dir = 1;
    for (int k = 0; k < cnt; k++) {
        r[k + team * cnt]->setXY(x[k] * dir, y[k]);
        r[k + team * cnt]->resetRobot();
    }
}
