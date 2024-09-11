#include "TankDrive.h"
#include <frc/motorcontrol/Spark.h>

TankDrive::TankDrive() : m_LeftSparkOne(3), m_LeftSparkTwo(2), m_RightSparkOne(1), m_RightSparkTwo(0) {
    //Set up all SPARK motor controllers. 
    //Since they have already been instantiated in the .h file, simply set them all to 0.
    
}
void TankDrive::SetLeftSpeed(double speed) {
    //Set the left SPARK controllers to the variable speed.
    
}
void TankDrive::SetRightSpeed(double speed) {
    //Set the right SPARK controllers to the variable speed.
    
    
}
double TankDrive::GetRightSpeed() {
    //Return the voltage of one of the right SPARK controllers.
}
double TankDrive::GetLeftSpeed() {
    //Return the voltage of one of the left SPARK controllers.
}
void TankDrive::StopDrive() {
    //Set all motor controllers to 0.
}
void TankDrive::Periodic() {
    
}