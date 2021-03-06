#pragma once

#include <string>

// selector configuration
#define HUE 46
#define DEFAULT 1
#define AUTONS "Right One Goal", "Left Ring", "Single AWP"

namespace selector
{

    extern int auton;
    const char *b[] = {AUTONS, ""};
    void init(int hue = HUE, int default_auton = DEFAULT, const char **autons = b);

}
