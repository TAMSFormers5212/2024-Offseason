#pragma once
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <frc/geometry/Translation2d.h>

#include <units/length.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/acceleration.h>
#include <units/math.h>
#include <units/velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>
// #include <units/base.h>
#include <numbers>
#include <string>
#include <vector>
#include <array>

namespace Autos {
    const char* const LINE = "Line Auto";
}

namespace OIConstants {
    constexpr int kDriverControllerPort = 0;
    constexpr int kOperatorControllerPort = 1;

    namespace Controller {
        constexpr int leftXAxis = 0;
        constexpr int leftYAxis = 1;
        constexpr int rightXAxis = 4;
        constexpr int rightYAxis = 5;

        constexpr int A = 1;
        constexpr int B = 2;
        constexpr int X = 3;
        constexpr int Y = 4;
        constexpr int leftBumper = 5;
        constexpr int rightBumper = 6;
        constexpr int rightTrigger = 3;
    }

    constexpr int kBreakBeamChannel = 0;
    constexpr int kIntakeMotor = 0;
    constexpr int kShooterMotor = 4;
}

namespace ShooterConstants {
    constexpr int leftMotor = 15;
    constexpr int rightMotor = 14;
    constexpr double pulleyRatio = 30.0 / 18.0;
    constexpr units::inch_t wheelDiameter = 4_in; // may change based on rpm

    //velocity pid
    constexpr double ksS = 0.1;
    constexpr double ksV = 0.1;
    constexpr double ksA = 0.0;

    //idk abt these values so making new ones
    //TASK: Find Values for ksP, ksI, ksD
    constexpr double ksP = 0.0005;
    constexpr double ksI = 0.0;
    constexpr double ksD = 0.0002;
    constexpr double ksFF = 0;

    constexpr units::revolutions_per_minute_t maxNeoRpm = 5700_rpm;
    constexpr units::revolutions_per_minute_t maxWheelRpm = 9000_rpm;//replace this value with whatever max andymark says
    constexpr units::feet_per_second_t maxExitVelocity = 90_fps;

    constexpr units::volt_t KlsS{ 0.16 }; // friction term`
    constexpr units::volt_t vKlsV{ 0.019 }; // velocity term
    constexpr units::meter_t aKlsV{ 1 };
    constexpr units::second_t sKlsV{ 1 };
    constexpr auto KlsV = vKlsV * sKlsV / aKlsV;


    constexpr units::volt_t KrsS{ 0.195 }; // friction term
    constexpr units::volt_t vKrsV{ 0.019 }; // velocity term
    constexpr units::meter_t aKrsV{ 1 };
    constexpr auto KrsV = vKrsV * sKlsV / aKrsV;

}
namespace MathConstants {
    constexpr double pi = 3.1415926535;
    constexpr double pi2 = 6.283185307;

    constexpr double gravityMeters = 9.81;
    constexpr double gravityFeet = 32.1741;

    constexpr double InchToCM = 2.54;
}