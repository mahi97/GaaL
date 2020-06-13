//
// Created by Mahi on 6/12/20.
//
#include "sslworld.hpp"
#include "iostream"
#include <vector>

int main() {
    auto* ssl = new SSLWorld();
    bool done = false;
    ssl->reset();
    std::vector<Action*> actions;
    actions.clear();
    auto action = new Action(0,0);
    actions.push_back(action);
    while (!done) {
        auto ob = ssl->getObservation(-1);
        auto& b = ob->robots[0][0];
        std::cout << b[0] << " " << b[1] << " " << b[2] << std::endl;
        action->vel[0] = action->vel[1] = action->vel[2] = ssl->framenum;
        ssl->act(actions);
        ssl->step();
        if (ssl->framenum > 100)
            ssl->reset();
    }
    return 0;
}