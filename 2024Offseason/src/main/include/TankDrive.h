
#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SubsystemBase.h>

#include <frc/motorcontrol/Spark.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelPositions.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/drive/DifferentialDrive.h>
#include <units/velocity.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkRelativeEncoder.h>
#include <rev/SparkPIDController.h>

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPRamseteController.h>

using namespace rev;
using namespace frc;


class TankDrive : public frc2::SubsystemBase {
public:
    TankDrive();

    inline const frc::Pose2d& getPose() const {
        return m_driveOdometry.GetPose();
    }

    inline void resetPose(const frc::Pose2d& pose) {
        m_driveOdometry.ResetPosition(0_deg, 0_m, 0_m, pose);
    }

    inline frc::ChassisSpeeds getRobotRelativeSpeeds() {
        return m_driveKinematics.ToChassisSpeeds(m_driveSpeeds);
    }

    void driveRobotRelative(const frc::ChassisSpeeds& speeds);

    void Periodic() override;
    void SetLeftSpeed(double speed);
    void SetRightSpeed(double speed);
    double GetLeftSpeed();
    double GetRightSpeed();
    void StopDrive();

    frc::DifferentialDrive m_drive{
        [&](double output) { SetLeftSpeed(output); },
        [&](double output) { SetRightSpeed(output); }
    };
private:
    Spark m_LeftSparkOne{ 3 };
    Spark m_LeftSparkTwo{ 2 };
    Spark m_RightSparkOne{ 1 };
    Spark m_RightSparkTwo{ 0 };

    frc::DifferentialDriveWheelSpeeds m_driveSpeeds;
    frc::DifferentialDriveWheelPositions m_drivePositions;

    frc::DifferentialDriveKinematics m_driveKinematics{ 30_in }; // TODO: get track width
    frc::DifferentialDriveOdometry m_driveOdometry{
        {}, // TODO: make odo simulate
        units::meter_t{0},
        units::meter_t{0}
    };
};