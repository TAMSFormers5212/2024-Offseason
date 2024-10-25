#include "Intake.h"

#include <cmath>
#include <iostream>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace rev;
using namespace std;

Intake::Intake(int motor) : m_intakeMotor(motor, CANSparkLowLevel::MotorType::kBrushless) {
    ResetMotor();
}

void Intake::ResetMotor() {
    m_intakeMotor.RestoreFactoryDefaults();
    m_intakeMotor.SetIdleMode(CANSparkBase::IdleMode::kCoast);
    m_intakeMotor.SetSmartCurrentLimit(20, 25);
}

void Intake::SetSpeed(double speed) {
    m_intakeMotor.Set(speed);
}

double Intake::GetSpeed() {
    return m_intakeMotor.Get();
}

bool Intake::isOn() {
    return on;
}

void Intake::setOn(bool on) {
    this->on = on;
}

void Intake::toggleOn() {
    setOn(!on);
}

void Intake::Periodic() {
    frc::SmartDashboard::PutNumber("Intake Speed", GetSpeed());
}
