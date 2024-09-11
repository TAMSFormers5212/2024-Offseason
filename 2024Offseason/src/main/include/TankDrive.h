
#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkPIDController.h>
#include <frc/motorcontrol/Spark.h>

using namespace rev;
using namespace frc;

class TankDrive : public frc2::SubsystemBase {
    public:
        TankDrive();
        void Periodic() override;
        void SetLeftSpeed(double speed);
        void SetRightSpeed(double speed);
        void StopDrive();
        double GetRightSpeed();
        double GetLeftSpeed();
    private:
        //Instantiate four SPARK motor controllers and make two variables for left speed and right speed.


};