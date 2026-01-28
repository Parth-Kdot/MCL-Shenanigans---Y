#pragma once

#include "pros/motors.hpp"

class Intake {
  public:
    Intake() = default;

    void telOP(bool intake, bool scoreTop, bool scoreMid, bool outtake, bool prime);
};
