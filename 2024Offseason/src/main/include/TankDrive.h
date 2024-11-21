
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
        // m_driveOdometry.ResetPosition(Rotation2d(0), 0, 0, pose);
    }

    inline frc::ChassisSpeeds getRobotRelativeSpeeds() {
        return m_driveKinematics.ToChassisSpeeds(m_driveSpeeds);
    }

    void driveRobotRelative(const frc::ChassisSpeeds &speeds);


    void Periodic() override;
    void SetLeftSpeed(double speed);
    void SetRightSpeed(double speed);
    void StopDrive();
    double GetRightSpeed();
    double GetLeftSpeed();

    void forward(units::second_t seconds, double speed);
private:
    frc::DifferentialDriveWheelSpeeds m_driveSpeeds;
    frc::DifferentialDriveWheelPositions m_drivePositions;

    frc::DifferentialDriveKinematics m_driveKinematics{30_in}; // TODO: get track width
    frc::DifferentialDriveOdometry m_driveOdometry{
        {}, // TODO: make odo simulate
        units::meter_t{0},
        units::meter_t{0}
    };

    Spark m_LeftSparkOne;
    Spark m_LeftSparkTwo;
    Spark m_RightSparkOne;
    Spark m_RightSparkTwo;
    //Instantiate four SPARK motor controllers and make two variables for left speed and right speed.
};