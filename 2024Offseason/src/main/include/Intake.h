
#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkPIDController.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/DigitalInput.h>

using namespace rev;
using namespace frc;

class Intake : public frc2::SubsystemBase {
public:
    Intake(int motor);
    void ResetMotor();
    void Periodic() override;
    void SetSpeed(double speed);
    double GetSpeed();

    bool isOn();
    void setOn(bool on);
    void toggleOn();
private:
    bool on = false;

    CANSparkMax m_intakeMotor;
    DigitalInput m_beamBreak{ OIConstants::kBreakBeamChannel };
};
