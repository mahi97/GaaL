//
// Created by Mahi on 6/12/20.
//

#ifndef COMMON_H
#define COMMON_H

#define MAX_ROBOT_COUNT 16 //don't change me
#define TEAM_COUNT 2 // me neither

struct Action {
    Action(int _team_id, int _robot_id) : team_id(_team_id), robot_id(_robot_id) {}
    const int team_id = -1;
    const int robot_id = -1;
    double vel[3]={};
    int spin = -1;
    double kick[2]{};
};
typedef double GAction[5]; // vel_x, vel_y, vel_w, kick_x, kick_y

struct Observation {
    double ball[6]{};
    double robots  [TEAM_COUNT][MAX_ROBOT_COUNT][6]{};
    int kick  [TEAM_COUNT][MAX_ROBOT_COUNT][2]{};
};
typedef double GObservation[TEAM_COUNT*MAX_ROBOT_COUNT + 1][8];
typedef double LObservation[8];

struct Replace {
    double ball[4]{};
    double robots  [TEAM_COUNT][MAX_ROBOT_COUNT][3]{};
};


#endif
