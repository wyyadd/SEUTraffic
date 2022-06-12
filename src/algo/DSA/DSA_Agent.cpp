//
// Created by wyyadd on 6/12/22.
//

#include "DSA_Agent.h"

namespace ALGO {
    void DSA_Agent::sendMessage(Message message, DSA_Agent *neighbour) {
       neighbour->getMailBox().push_back(message);
    }
} // ALGO