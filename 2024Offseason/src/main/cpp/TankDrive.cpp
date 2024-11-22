#include "TankDrive.h"

#include <frc/motorcontrol/Spark.h>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Timer.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPLTVController.h>

using namespace pathplanner;


TankDrive::TankDrive() {
    m_LeftSparkOne.AddFollower(m_LeftSparkTwo);
    m_RightSparkOne.AddFollower(m_RightSparkTwo);
    m_LeftSparkOne.SetInverted(true);


    AutoBuilder::configureRamsete(
        [this]() { return getPose(); },
        [this](frc::Pose2d pose) { resetPose(pose); },
        [this]() { return getRobotRelativeSpeeds(); },
        [this](frc::ChassisSpeeds speeds) { driveRobotRelative(speeds); },
        ReplanningConfig(),
        []() {
            return false;
        },
        this
    );

    //Set up all SPARK motor controllers. 
    //Since they have already been instantiated in the .h file, simply set them all to 0.
}

//Set the left SPARK controllers to the variable speed.
void TankDrive::SetLeftSpeed(double speed) {
    m_LeftSparkOne.Set(speed);
}

//Set the right SPARK controllers to the variable speed.
void TankDrive::SetRightSpeed(double speed) {
    m_RightSparkOne.Set(speed);
}

//Return the voltage of one of the left SPARK controllers.
double TankDrive::GetLeftSpeed() {
    return m_LeftSparkOne.Get();
}

//Return the voltage of one of the right SPARK controllers.
double TankDrive::GetRightSpeed() {
    return m_RightSparkOne.Get();
}

//Set all motor controllers to 0.
void TankDrive::StopDrive() {
    SetRightSpeed(0);
    SetLeftSpeed(0);
}

constexpr units::meters_per_second_t MAX_VEL = units::meters_per_second_t(1.0);
constexpr double M_PWM_PER_SEC = 1.0;

frc::Timer dtimer;

void TankDrive::driveRobotRelative(const frc::ChassisSpeeds& speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = m_driveKinematics.ToWheelSpeeds(speeds);
    m_drive.TankDrive(wheelSpeeds.left / MAX_VEL, wheelSpeeds.right / MAX_VEL);
}

units::meter_t leftMeters = 0_m;
units::meter_t rightMeters = 0_m;

void TankDrive::Periodic() {
    leftMeters += units::meter_t(GetLeftSpeed() * M_PWM_PER_SEC * dtimer.Get().value());
    rightMeters += units::meter_t(GetRightSpeed() * M_PWM_PER_SEC * dtimer.Get().value());
    m_driveOdometry.Update(0_deg,
        leftMeters,
        rightMeters
    );
    dtimer.Reset();
    dtimer.Start();

    frc::SmartDashboard::PutNumber("Drive Left Vel", GetLeftSpeed());
    frc::SmartDashboard::PutNumber("Drive Right Vel", GetRightSpeed());
    auto pose = m_driveOdometry.GetPose();
    frc::SmartDashboard::PutNumber("Drive X", pose.X().value());
    frc::SmartDashboard::PutNumber("Drive Y", pose.Y().value());
    frc::SmartDashboard::PutNumber("Drive H", pose.Rotation().Degrees().value());
}
