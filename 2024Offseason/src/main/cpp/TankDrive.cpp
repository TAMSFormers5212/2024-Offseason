#include "TankDrive.h"
#include <frc/motorcontrol/Spark.h>

TankDrive::TankDrive() : m_LeftSparkOne(3), m_LeftSparkTwo(2), m_RightSparkOne(1), m_RightSparkTwo(0) {
    m_LeftSparkOne.Set(0);
    m_LeftSparkTwo.Set(0);
    m_RightSparkOne.Set(0);
    m_RightSparkTwo.Set(0);
    //Set up all SPARK motor controllers. 
    //Since they have already been instantiated in the .h file, simply set them all to 0.
    
}
void TankDrive::SetLeftSpeed(double speed) {
    m_LeftSparkOne.Set(speed);
    m_LeftSparkTwo.Set(speed);

    //Set the left SPARK controllers to the variable speed.
    
}
void TankDrive::SetRightSpeed(double speed) {
    m_RightSparkOne.Set(speed);
    m_RightSparkTwo.Set(speed);
    //Set the right SPARK controllers to the variable speed.
    
    
}
double TankDrive::GetRightSpeed() {
    return m_RightSparkOne.Get();
    //Return the voltage of one of the right SPARK controllers.
}
double TankDrive::GetLeftSpeed() {
    return m_LeftSparkOne.Get();
    //Return the voltage of one of the left SPARK controllers.
}
void TankDrive::StopDrive() {
    SetRightSpeed(0);
    SetLeftSpeed(0);
    //Set all motor controllers to 0.
}
void TankDrive::Periodic() {
    
}