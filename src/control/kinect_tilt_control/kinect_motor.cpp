#include <string>
#include <unistd.h>
#include "KinectMotor.h"

int main(int argc, char *argv[])
{
    if (argc > 2) {
        fprintf(stderr, "Usage: %s [-31 - 31]\n", argv[0]);
        exit(1);
    }

    if (argc == 1) { // strut your stuff
        KinectMotor motor;

        if (!motor.Open()) {// Open motor device
            return 1;
        }

        printf("MOVIN TO THE MAGICAL 31\n");

        motor.Move(31); // move it up to 31 degree
        sleep(1);

        printf("MOVIN TO THE MAGICAL -31\n");
        motor.Move(-31); // move it down to 31 degree
        sleep(1);

        printf("MOVIN TO THE MAGICAL 0\n");

        motor.Move(0);
    }
    else {
        int val = std::stoi(argv[1]);
        auto clamp = [](int i, int min, int max) { if (i < min) return min; else if (i > max) return max; else return i; };

        const int tiltmin = -31;
        const int tiltmax = 31;

        printf("clamping %d to [%d, %d]\n", val, tiltmin, tiltmax);

        val = clamp(val, tiltmin, tiltmax);

        KinectMotor motor;

        if (!motor.Open()) {// Open motor device
            return 1;
        }

        motor.Move(val); // move it up to 31 degree
        sleep(1);
    }

    return 0;
}