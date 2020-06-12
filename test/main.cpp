//
// Created by Mahi on 6/12/20.
//
#include "sslworld.h"
#include "iostream"

int main() {
    auto* ssl = new SSLWorld(0.016);
    bool done = true;
    Action actions[6]{};
    for (int i = 0; i < 6; i++) {
        actions[i].robot_id = i % 3;
        actions[i].team_id = (i <  3? 0 : 1);
        actions[i].use_wheel = false;
        actions[i].vel[0] = actions[i].vel[0] = actions[i].vel[0] = 100;
    }

    while (done) {
        auto ob = ssl->getObservation(-1);
        auto& b = ob->robots[0][0];
        std::cout << b[0] << " " << b[1] << " " << b[2] << std::endl;
        ssl->act(actions, 5);
        ssl->step();
    }
    return 0;
}