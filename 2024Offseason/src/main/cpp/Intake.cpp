#include "Intake.h"

#include <cmath>
#include <iostream>
#include <frc/DigitalInput.h>
#include <frc/smartdashboard/SmartDashboard.h>

using namespace rev;
using namespace std;

Intake::Intake(int motor) : m_intakeMotor(motor, CANSparkLowLevel::MotorType::kBrushless) {
    resetMotor();
}

void Intake::resetMotor() {
    m_intakeMotor.RestoreFactoryDefaults();
    m_intakeMotor.SetIdleMode(CANSparkBase::IdleMode::kCoast);
    m_intakeMotor.SetSmartCurrentLimit(20, 25);
}

void Intake::setSpeed(double speed) {
    this->speed = speed;
    m_intakeMotor.Set(this->speed);
}

double Intake::getSpeed() {
    return speed;
}

bool Intake::isOn() {
    return speed != 0.0;
}

void Intake::toggle() {
    setSpeed(speed != 0 ? 0 : 0.5);
}

void Intake::toggleReverse() {
    setSpeed(speed != 0 ? 0 : -0.5);
}

void Intake::Periodic() {
    frc::SmartDashboard::PutNumber("Intake Speed", getSpeed());
}
