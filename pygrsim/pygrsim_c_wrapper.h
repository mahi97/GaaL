//
// Created by Mahi on 6/12/20.
//

#ifndef GYMSIM_PYGRSIM_C_WRAPPER_H
#define GYMSIM_PYGRSIM_C_WRAPPER_H

#include <vector>
#include <string.h>
#include <sslworld.hpp>
#include <common.hpp>

extern "C" {
    SSLWorld *SSLWorld_new() { return new SSLWorld(); }
    void SSL_del(SSLWorld *ssl) { delete ssl; }
    Observation* step(SSLWorld* ssl, const std::vector<Action*>& actions) {
        ssl->act(actions);
        ssl->step();
        return ssl->getObservation(-1);
    }

    Observation* reset(SSLWorld* ssl) {
        ssl->reset();
        return ssl->getObservation(-1);
    }


}
#endif //GYMSIM_PYGRSIM_C_WRAPPER_H
