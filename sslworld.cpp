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

#include "sslworld.h"

#define ROBOT_GRAY 0.4
#define WHEEL_COUNT 4
#define CONV_UNIT(x) ((int)(1000*(x)))

using namespace std;

SSLWorld* _w;

bool ballCallBack(dGeomID o1, dGeomID o2, PSurface *s, int /*robots_count*/);
bool wheelCallBack(dGeomID o1, dGeomID o2, PSurface *s, int /*robots_count*/);


SSLWorld::SSLWorld(const dReal &_dt) : dt(_dt) {

    _w = this;
    cfg = new ConfigWidget();

    auto* form1 = new RobotsFormation(1, cfg);
    auto* form2 = form1;
    framenum = 0;
    last_dt = -1;
    p = new PWorld(0.05, 9.81f, cfg->Robots_Count());

    ground = new PGround(cfg->Field_Rad(), cfg->Field_Length(), cfg->Field_Width(), cfg->Field_Penalty_Depth(),
                         cfg->Field_Penalty_Width(), cfg->Field_Penalty_Point(), cfg->Field_Line_Width(), 0);

    ball = new PBall(0, 0, 0.5, cfg->BallRadius(), cfg->BallMass(), 1, 0.7, 0);
    // Bounding walls

    const double thick = cfg->Wall_Thickness();
    const double increment = cfg->Field_Margin() + cfg->Field_Referee_Margin() + thick / 2;
    const double pos_x = cfg->Field_Length() / 2.0 + increment;
    const double pos_y = cfg->Field_Width() / 2.0 + increment;
    const double pos_z = 0.0;
    const double siz_x = 2.0 * pos_x;
    const double siz_y = 2.0 * pos_y;
    const double siz_z = 0.4;
    const double tone = 1.0;

    walls[0] = new PFixedBox(thick / 2, pos_y, pos_z,
                             siz_x, thick, siz_z,
                             tone, tone, tone);

    walls[1] = new PFixedBox(-thick / 2, -pos_y, pos_z,
                             siz_x, thick, siz_z,
                             tone, tone, tone);

    walls[2] = new PFixedBox(pos_x, -thick / 2, pos_z,
                             thick, siz_y, siz_z,
                             tone, tone, tone);

    walls[3] = new PFixedBox(-pos_x, thick / 2, pos_z,
                             thick, siz_y, siz_z,
                             tone, tone, tone);

    // Goal walls

    const double gthick = cfg->Goal_Thickness();
    const double gpos_x = (cfg->Field_Length() + gthick) / 2.0 + cfg->Goal_Depth();
    const double gpos_y = (cfg->Goal_Width() + gthick) / 2.0;
    const double gpos_z = cfg->Goal_Height() / 2.0;
    const double gsiz_x = cfg->Goal_Depth() + gthick;
    const double gsiz_y = cfg->Goal_Width();
    const double gsiz_z = cfg->Goal_Height();
    const double gpos2_x = (cfg->Field_Length() + gsiz_x) / 2.0;

    walls[4] = new PFixedBox(gpos_x, 0.0, gpos_z,
                             gthick, gsiz_y, gsiz_z,
                             tone, tone, tone);

    walls[5] = new PFixedBox(gpos2_x, -gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z,
                             tone, tone, tone);

    walls[6] = new PFixedBox(gpos2_x, gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z,
                             tone, tone, tone);

    walls[7] = new PFixedBox(-gpos_x, 0.0, gpos_z,
                             gthick, gsiz_y, gsiz_z,
                             tone, tone, tone);

    walls[8] = new PFixedBox(-gpos2_x, -gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z,
                             tone, tone, tone);

    walls[9] = new PFixedBox(-gpos2_x, gpos_y, gpos_z,
                             gsiz_x, gthick, gsiz_z,
                             tone, tone, tone);

    p->addObject(ground);
    p->addObject(ball);
    for (auto &wall : walls) p->addObject(wall);
    const int wheeltexid = 4 * cfg->Robots_Count() + 12 + 1; //37 for 6 robots


    cfg->robotSettings = cfg->blueSettings;
    for (int k = 0; k < cfg->Robots_Count(); k++) {
        float a1 = -form1->x[k];
        float a2 = form1->y[k];
        float a3 = ROBOT_START_Z(cfg);
        robots[k] = new SimRobot(p,
                                 ball,
                                 cfg,
                                 -form1->x[k],
                                 form1->y[k],
                                 ROBOT_START_Z(cfg),
                                 ROBOT_GRAY,
                                 ROBOT_GRAY,
                                 ROBOT_GRAY,
                                 k + 1,
                                 wheeltexid,
                                 1);
    }
    cfg->robotSettings = cfg->yellowSettings;
    for (int k = 0; k < cfg->Robots_Count(); k++)
        robots[k + cfg->Robots_Count()] = new SimRobot(p, ball, cfg, form2->x[k], form2->y[k], ROBOT_START_Z(cfg),
                                                       ROBOT_GRAY, ROBOT_GRAY, ROBOT_GRAY, k + cfg->Robots_Count() + 1,
                                                       wheeltexid, -1);//XXX

    p->initAllObjects();

    //Surfaces
    PSurface ballwithwall;
    ballwithwall.surface.mode = dContactBounce | dContactApprox1;// | dContactSlip1;
    ballwithwall.surface.mu = 1;//fric(cfg->ballfriction());
    ballwithwall.surface.bounce = cfg->BallBounce();
    ballwithwall.surface.bounce_vel = cfg->BallBounceVel();
    ballwithwall.surface.slip1 = 0;//cfg->ballslip();

    PSurface wheelswithground;
    PSurface *ball_ground = p->createSurface(ball, ground);
    ball_ground->surface = ballwithwall.surface;
    ball_ground->callback = ballCallBack;

    PSurface ballwithkicker;
    ballwithkicker.surface.mode = dContactApprox1;
    ballwithkicker.surface.mu = fric(cfg->robotSettings.Kicker_Friction);
    ballwithkicker.surface.slip1 = 5;

    for (auto &wall : walls) p->createSurface(ball, wall)->surface = ballwithwall.surface;

    for (int k = 0; k < 2 * cfg->Robots_Count(); k++) {
        p->createSurface(robots[k]->chassis, ground);
        for (auto &wall : walls)
            p->createSurface(robots[k]->chassis, wall);
        p->createSurface(robots[k]->dummy, ball);
        //p->createSurface(robots[k]->chassis,ball);
        p->createSurface(robots[k]->kicker->box, ball)->surface = ballwithkicker.surface;
        for (auto &wheel : robots[k]->wheels) {
            p->createSurface(wheel->cyl, ball);
            PSurface *w_g = p->createSurface(wheel->cyl, ground);
            w_g->surface = wheelswithground.surface;
            w_g->usefdir1 = true;
            w_g->callback = wheelCallBack;
        }
        for (int j = k + 1; j < 2 * cfg->Robots_Count(); j++) {
            if (k != j) {
                p->createSurface(robots[k]->dummy,
                                 robots[j]->dummy); //seams ode doesn't understand cylinder-cylinder contacts, so I used spheres
                p->createSurface(robots[k]->chassis, robots[j]->kicker->box);
            }
        }
    }
    sendGeomCount = 0;
    new char[65536];

    // initialize robot state
    for (int team = 0; team < 2; ++team) {
        for (int i = 0; i < MAX_ROBOT_COUNT; ++i) {
            lastInfraredState[team][i] = false;
            lastKickState[team][i] = NO_KICK;
        }
    }
}

int SSLWorld::robotIndex(int robot, int team) {
    if (robot >= cfg->Robots_Count()) return -1;
    return robot + team * cfg->Robots_Count();
}

SSLWorld::~SSLWorld() {
    std::cout << "Here";
    delete p;
}

void SSLWorld::step() {
    int ballCollisionTry = 5;
    for (int kk = 0; kk < ballCollisionTry; kk++) {
        const dReal *ballvel = dBodyGetLinearVel(ball->body);
        dReal ballspeed = ballvel[0] * ballvel[0] + ballvel[1] * ballvel[1] + ballvel[2] * ballvel[2];
        ballspeed = sqrt(ballspeed);
        dReal ballfx = 0, ballfy = 0, ballfz = 0;
        dReal balltx = 0, ballty = 0, balltz = 0;
        if (ballspeed >= 0.01) {
            dReal fk = cfg->BallFriction() * cfg->BallMass() * cfg->Gravity();
            ballfx = -fk * ballvel[0] / ballspeed;
            ballfy = -fk * ballvel[1] / ballspeed;
            ballfz = -fk * ballvel[2] / ballspeed;
            balltx = -ballfy * cfg->BallRadius();
            ballty = ballfx * cfg->BallRadius();
            balltz = 0;
            dBodyAddTorque(ball->body, balltx, ballty, balltz);
        }
        dBodyAddForce(ball->body, ballfx, ballfy, ballfz);
        if (dt == 0) dt = last_dt;
        else last_dt = dt;

        selected = -1;
        p->step(dt / ballCollisionTry);
    }

    int best_k = -1;
    dReal best_dist = 1e8;
    dReal xyz[3], hpr[3];
    if (selected == -2) {
        best_k = -2;
        dReal bx, by, bz;
        ball->getBodyPosition(bx, by, bz);
        best_dist = (bx - xyz[0]) * (bx - xyz[0])
                    + (by - xyz[1]) * (by - xyz[1])
                    + (bz - xyz[2]) * (bz - xyz[2]);
    }
    for (int k = 0; k < cfg->Robots_Count() * 2; k++) {
        if (robots[k]->selected) {
            dReal dist = (robots[k]->select_x - xyz[0]) * (robots[k]->select_x - xyz[0])
                         + (robots[k]->select_y - xyz[1]) * (robots[k]->select_y - xyz[1])
                         + (robots[k]->select_z - xyz[2]) * (robots[k]->select_z - xyz[2]);
            if (dist < best_dist) {
                best_dist = dist;
                best_k = k;
            }
        }
        robots[k]->chassis->setColor(ROBOT_GRAY, ROBOT_GRAY, ROBOT_GRAY);
    }
    if (best_k >= 0) robots[best_k]->chassis->setColor(ROBOT_GRAY * 2, ROBOT_GRAY * 1.5, ROBOT_GRAY * 1.5);
    selected = best_k;
    ball->tag = -1;
    for (int k = 0; k < cfg->Robots_Count() * 2; k++) {
        robots[k]->step();
        robots[k]->selected = false;
    }
    dMatrix3 R;

    framenum++;
}

void SSLWorld::act(const Action actions[], const int &size) {

    for (int i = 0; i < size; i++) {
        const auto &action = actions[i];
        int team = action.team_id;
        int k = action.robot_id;
        int id = robotIndex(k, team);
        if ((id < 0) || (id >= cfg->Robots_Count() * 2)) continue;

        if (action.use_wheel) for (int j = 0; j < WHEEL_COUNT; j++) robots[id]->setSpeed(j, action.wheel[j]);
        else robots[id]->setSpeed(action.vel[0], action.vel[1], action.vel[2]);

        dReal kickx = action.kick[0];
        dReal kickz = action.kick[1];
        if (kickx > 0.0001 || kickz > 0.0001) robots[id]->kicker->kick(kickx, kickz);
        robots[id]->kicker->setRoller(action.spin);
    }
}


void SSLWorld::reset() {
//    if (packet.has_replacement()) {
//        for (int i = 0; i < packet.replacement().robots_size(); i++) {
//            int team = 0;
//            if (packet.replacement().robots(i).has_yellowteam()) {
//                if (packet.replacement().robots(i).yellowteam())
//                    team = 1;
//            }
//            if (!packet.replacement().robots(i).has_id()) continue;
//            int k = packet.replacement().robots(i).id();
//            dReal x = 0, y = 0, dir = 0;
//            bool turnon = true;
//            if (packet.replacement().robots(i).has_x()) x = packet.replacement().robots(i).x();
//            if (packet.replacement().robots(i).has_y()) y = packet.replacement().robots(i).y();
//            if (packet.replacement().robots(i).has_dir()) dir = packet.replacement().robots(i).dir();
//            if (packet.replacement().robots(i).has_turnon()) turnon = packet.replacement().robots(i).turnon();
//            int id = robotIndex(k, team);
//            if ((id < 0) || (id >= cfg->Robots_Count() * 2)) continue;
//            robots[id]->setXY(x, y);
//            robots[id]->resetRobot();
//            robots[id]->setDir(dir);
//            robots[id]->on = turnon;
//        }
//        if (packet.replacement().has_ball()) {
//            dReal x = 0, y = 0, vx = 0, vy = 0;
//            if (packet.replacement().ball().has_x()) x = packet.replacement().ball().x();
//            if (packet.replacement().ball().has_y()) y = packet.replacement().ball().y();
//            if (packet.replacement().ball().has_vx()) vx = packet.replacement().ball().vx();
//            if (packet.replacement().ball().has_vy()) vy = packet.replacement().ball().vy();
//            ball->setBodyPosition(x, y, cfg->BallRadius() * 1.2);
//            dBodySetLinearVel(ball->body, vx, vy, 0);
//            dBodySetAngularVel(ball->body, 0, 0, 0);
//        }
//    }
}


dReal normalizeAngle(dReal a) {
    if (a > 180) return -360 + a;
    if (a < -180) return 360 + a;
    return a;
}

bool SSLWorld::visibleInCam(int id, double x, double y) {

    if (id == -1) return true;
    id %= 4;
    if (id == 0) {
        if (x > -0.2 && y > -0.2) return true;
    }
    if (id == 1) {
        if (x > -0.2 && y < 0.2) return true;
    }
    if (id == 2) {
        if (x < 0.2 && y < 0.2) return true;
    }
    if (id == 3) {
        if (x < 0.2 && y > -0.2) return true;
    }
    return false;
}

Observation* SSLWorld::getObservation(const int &cam_id) {
    auto* obs = new Observation();
    dReal x, y, z, dir, k;
    ball->getBodyPosition(x, y, z);

    dReal dev_x = cfg->noiseDeviation_x();
    dReal dev_y = cfg->noiseDeviation_y();
    dReal dev_a = cfg->noiseDeviation_angle();
    if (!cfg->noise()) dev_x = dev_y = dev_a = 0;

    if (!cfg->vanishing() || (rand0_1() > cfg->ball_vanishing())) {
        if (visibleInCam(cam_id, x, y)) {
            obs->ball[0] = rand_notrig(x * 1000.0f, dev_x);
            obs->ball[1] = rand_notrig(y * 1000.0f, dev_y);
            obs->ball[2] = z * 1000.0f;
        }
        for (int i = 0; i < cfg->Robots_Count() * 2; i++) {
            if (!robots[i]->on) continue;
            robots[i]->getXY(x, y);
            dir = robots[i]->getDir(k);
            // reset when the robot has turned over
//            if (k < 0.9) {
//                robots[i]->resetRobot();
//            }
            if (visibleInCam(cam_id, x, y)) {
                int team_id = 0;
                if (i >= cfg->Robots_Count()) team_id = 1;
                obs->robots[team_id][i %  cfg->Robots_Count()][0] = rand_notrig(x * 1000.0f, dev_x);
                obs->robots[team_id][i %  cfg->Robots_Count()][1] = rand_notrig(y * 1000.0f, dev_y);
                obs->robots[team_id][i %  cfg->Robots_Count()][2] = normalizeAngle(rand_notrig(dir, dev_a)) * M_PI / 180.0f;
                obs->kick[team_id][i %  cfg->Robots_Count()][0] = robots[i]->kicker->isTouchingBall();
                obs->kick[team_id][i %  cfg->Robots_Count()][1] = robots[i]->kicker->isKicking();
            }
        }
    }
    return obs;
}

void SSLWorld::addFieldLinesArcs(SSL_GeometryFieldSize *field) {
    const double kFieldLength = CONV_UNIT(cfg->Field_Length());
    const double kFieldWidth = CONV_UNIT(cfg->Field_Width());
    const double kGoalWidth = CONV_UNIT(cfg->Goal_Width());
    const double kGoalDepth = CONV_UNIT(cfg->Goal_Depth());
    const double kBoundaryWidth = CONV_UNIT(cfg->Field_Referee_Margin());
    const double kCenterRadius = CONV_UNIT(cfg->Field_Rad());
    const double kLineThickness = CONV_UNIT(cfg->Field_Line_Width());
    const double kPenaltyDepth = CONV_UNIT(cfg->Field_Penalty_Depth());
    const double kPenaltyWidth = CONV_UNIT(cfg->Field_Penalty_Width());

    const double kXMax = (kFieldLength - 2 * kLineThickness) / 2;
    const double kXMin = -kXMax;
    const double kYMax = (kFieldWidth - kLineThickness) / 2;
    const double kYMin = -kYMax;
    const double penAreaX = kXMax - kPenaltyDepth;
    const double penAreaY = kPenaltyWidth / 2.0;

    // Field lines
    addFieldLine(field, "TopTouchLine", kXMin - kLineThickness / 2, kYMax, kXMax + kLineThickness / 2, kYMax,
                 kLineThickness);
    addFieldLine(field, "BottomTouchLine", kXMin - kLineThickness / 2, kYMin, kXMax + kLineThickness / 2, kYMin,
                 kLineThickness);
    addFieldLine(field, "LeftGoalLine", kXMin, kYMin, kXMin, kYMax, kLineThickness);
    addFieldLine(field, "RightGoalLine", kXMax, kYMin, kXMax, kYMax, kLineThickness);
    addFieldLine(field, "HalfwayLine", 0, kYMin, 0, kYMax, kLineThickness);
    addFieldLine(field, "CenterLine", kXMin, 0, kXMax, 0, kLineThickness);
    addFieldLine(field, "LeftPenaltyStretch", kXMin + kPenaltyDepth - kLineThickness / 2, -kPenaltyWidth / 2,
                 kXMin + kPenaltyDepth - kLineThickness / 2, kPenaltyWidth / 2, kLineThickness);
    addFieldLine(field, "RightPenaltyStretch", kXMax - kPenaltyDepth + kLineThickness / 2, -kPenaltyWidth / 2,
                 kXMax - kPenaltyDepth + kLineThickness / 2, kPenaltyWidth / 2, kLineThickness);

    addFieldLine(field, "RightGoalTopLine", kXMax, kGoalWidth / 2, kXMax + kGoalDepth, kGoalWidth / 2,
                 kLineThickness);
    addFieldLine(field, "RightGoalBottomLine", kXMax, -kGoalWidth / 2, kXMax + kGoalDepth, -kGoalWidth / 2,
                 kLineThickness);
    addFieldLine(field, "RightGoalDepthLine", kXMax + kGoalDepth - kLineThickness / 2, -kGoalWidth / 2,
                 kXMax + kGoalDepth - kLineThickness / 2, kGoalWidth / 2, kLineThickness);

    addFieldLine(field, "LeftGoalTopLine", -kXMax, kGoalWidth / 2, -kXMax - kGoalDepth, kGoalWidth / 2,
                 kLineThickness);
    addFieldLine(field, "LeftGoalBottomLine", -kXMax, -kGoalWidth / 2, -kXMax - kGoalDepth, -kGoalWidth / 2,
                 kLineThickness);
    addFieldLine(field, "LeftGoalDepthLine", -kXMax - kGoalDepth + kLineThickness / 2, -kGoalWidth / 2,
                 -kXMax - kGoalDepth + kLineThickness / 2, kGoalWidth / 2, kLineThickness);

    addFieldLine(field, "LeftFieldLeftPenaltyStretch", kXMin, kPenaltyWidth / 2, kXMin + kPenaltyDepth,
                 kPenaltyWidth / 2, kLineThickness);
    addFieldLine(field, "LeftFieldRightPenaltyStretch", kXMin, -kPenaltyWidth / 2, kXMin + kPenaltyDepth,
                 -kPenaltyWidth / 2, kLineThickness);
    addFieldLine(field, "RightFieldLeftPenaltyStretch", kXMax, -kPenaltyWidth / 2, kXMax - kPenaltyDepth,
                 -kPenaltyWidth / 2, kLineThickness);
    addFieldLine(field, "RightFieldRightPenaltyStretch", kXMax, kPenaltyWidth / 2, kXMax - kPenaltyDepth,
                 kPenaltyWidth / 2, kLineThickness);

    // Field arcs
    addFieldArc(field, "CenterCircle", 0, 0, kCenterRadius - kLineThickness / 2, 0, 2 * M_PI, kLineThickness);
}

Vector2f *SSLWorld::allocVector(float x, float y) {
    auto *vec = new Vector2f();
    vec->set_x(x);
    vec->set_y(y);
    return vec;
}

void
SSLWorld::addFieldLine(SSL_GeometryFieldSize *field, const std::string &name, float p1_x, float p1_y, float p2_x,
                       float p2_y, float thickness) {
    SSL_FieldLineSegment *line = field->add_field_lines();
    line->set_name(name.c_str());
    line->mutable_p1()->set_x(p1_x);
    line->mutable_p1()->set_y(p1_y);
    line->mutable_p2()->set_x(p2_x);
    line->mutable_p2()->set_y(p2_y);
    line->set_thickness(thickness);
}

void
SSLWorld::addFieldArc(SSL_GeometryFieldSize *field, const std::string &name, float c_x, float c_y, float radius,
                      float a1, float a2, float thickness) {
    SSL_FieldCicularArc *arc = field->add_field_arcs();
    arc->set_name(name.c_str());
    arc->mutable_center()->set_x(c_x);
    arc->mutable_center()->set_y(c_y);
    arc->set_radius(radius);
    arc->set_a1(a1);
    arc->set_a2(a2);
    arc->set_thickness(thickness);
}

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

void RobotsFormation::loadFromFile(const QString &filename) {
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return;
    QTextStream in(&file);
    int k;
    for (k = 0; k < cfg->Robots_Count(); k++) x[k] = y[k] = 0;
    k = 0;
    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList list = line.split(",");
        if (list.count() >= 2) {
            x[k] = list[0].toFloat();
            y[k] = list[1].toFloat();
        } else if (list.count() == 1) {
            x[k] = list[0].toFloat();
        }
        if (k == cfg->Robots_Count() - 1) break;
        k++;
    }
}

void RobotsFormation::resetRobots(SimRobot **r, int team) {
    dReal dir = -1;
    if (team == 1) dir = 1;
    for (int k = 0; k < cfg->Robots_Count(); k++) {
        r[k + team * cfg->Robots_Count()]->setXY(x[k] * dir, y[k]);
        r[k + team * cfg->Robots_Count()]->resetRobot();
    }
}


bool wheelCallBack(dGeomID o1, dGeomID o2, PSurface *s, int /*robots_count*/) {
    //s->id2 is ground
    const dReal *r; //wheels rotation matrix
    //const dReal* p; //wheels rotation matrix
    if ((o1 == s->id1) && (o2 == s->id2)) {
        r = dBodyGetRotation(dGeomGetBody(o1));
        //p=dGeomGetPosition(o1);//never read
    } else if ((o1 == s->id2) && (o2 == s->id1)) {
        r = dBodyGetRotation(dGeomGetBody(o2));
        //p=dGeomGetPosition(o2);//never read
    } else {
        //XXX: in this case we dont have the rotation
        //     matrix, thus we must return
        return false;
    }

    s->surface.mode = dContactFDir1 | dContactMu2 | dContactApprox1 | dContactSoftCFM;
    s->surface.mu = fric(_w->cfg->robotSettings.WheelPerpendicularFriction);
    s->surface.mu2 = fric(_w->cfg->robotSettings.WheelTangentFriction);
    s->surface.soft_cfm = 0.002;

    dVector3 v = {0, 0, 1, 1};
    dVector3 axis;
    dMultiply0(axis, r, v, 4, 3, 1);
    dReal l = sqrt(axis[0] * axis[0] + axis[1] * axis[1]);
    s->fdir1[0] = axis[0] / l;
    s->fdir1[1] = axis[1] / l;
    s->fdir1[2] = 0;
    s->fdir1[3] = 0;
    s->usefdir1 = true;
    return true;
}

bool ballCallBack(dGeomID o1, dGeomID o2, PSurface *s, int /*robots_count*/) {
    if (_w->ball->tag != -1) //spinner adjusting
    {
        dReal x, y, z;
        _w->robots[_w->ball->tag]->chassis->getBodyDirection(x, y, z);
        s->fdir1[0] = x;
        s->fdir1[1] = y;
        s->fdir1[2] = 0;
        s->fdir1[3] = 0;
        s->usefdir1 = true;
        s->surface.mode = dContactMu2 | dContactFDir1 | dContactSoftCFM;
        s->surface.mu = _w->cfg->BallFriction();
        s->surface.mu2 = 0.5;
        s->surface.soft_cfm = 0.002;
    }
    return true;
}