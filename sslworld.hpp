/*
grSim - RoboCup Small Size Soccer Robots Simulator
Copyright (C) 2011, Parsian Robotic Center (eew.aut.ac.ir/~parsian/grsim)

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SSLWORLD_H
#define SSLWORLD_H
#include "helper.h"
#include "physics/pworld.h"
#include "physics/pball.h"
#include "physics/pground.h"
#include "physics/pfixedbox.h"
#include "physics/pray.h"
#include "configwidget.h"
#include "robot.h"
#include "messages_robocup_ssl_wrapper.pb.h"
#include "messages_robocup_ssl_robot_status.pb.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include <string>
#include "common.hpp"
#include "robotsformation.h"
#define WALL_COUNT 10
#include <QtGlobal>
#include <QDebug>
#include <QtCore/QFile>

class RobotsFormation;

class SSLWorld
{
public:
    SSLWorld();
    ~SSLWorld();
    void step();
    void addFieldLinesArcs(SSL_GeometryFieldSize *field);
    static Vector2f* allocVector(float x, float y);
    static void addFieldLine(SSL_GeometryFieldSize *field, const std::string &name, float p1_x, float p1_y, float p2_x, float p2_y, float thickness);
    static void addFieldArc(SSL_GeometryFieldSize *field, const std::string &name, float c_x, float c_y, float radius, float a1, float a2, float thickness);
    static bool visibleInCam(int id, double x, double y);
    int  robotIndex(int robot,int team);

    ConfigWidget* cfg;
    SimRobot* robots[MAX_ROBOT_COUNT*2]{};
    PBall* ball;
    PWorld* p;
    PGround* ground;
    PFixedBox* walls[WALL_COUNT]{};
    int selected{};
    int sendGeomCount;
    void act(const std::vector<Action*>& actions);
//    void reset(const Replace* replace);
    void reset();
    Observation* getObservation(const int& cam_id);
    int framenum;

private:
    dReal last_dt;
    char packet[200]{};
    bool lastInfraredState[TEAM_COUNT][MAX_ROBOT_COUNT]{};
    KickStatus lastKickState[TEAM_COUNT][MAX_ROBOT_COUNT]{};
    dReal dt;
    int robot_cnt;
    int bcnt;
    int ycnt;

};



#endif // SSLWORLD_H
