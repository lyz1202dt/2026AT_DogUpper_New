#pragma once

#include <OsqpEigen/OsqpEigen.h>

class MPC{
    public:
    MPC();
    ~MPC();
    bool update();
};