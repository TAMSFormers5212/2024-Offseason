
#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkPIDController.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/DigitalInput.h>
#include "Constants.h"

using namespace rev;
using namespace frc;

class Intake : public frc2::SubsystemBase {
public:
    Intake(int motor);
    void Periodic() override;

    void resetMotor();
    void setSpeed(double speed);
    double getSpeed();

    bool isOn();
    void toggle();
    void toggleReverse();
private:
    double speed = 0.0;

    CANSparkMax m_intakeMotor;
    DigitalInput m_beamBreak{ OIConstants::kBreakBeamChannel };
};
