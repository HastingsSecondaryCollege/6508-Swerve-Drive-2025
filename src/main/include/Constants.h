#include <numbers>
#include <frc/TimedRobot.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

// Global Constants

namespace DriveConstants {

} //Drive Constants

namespace ElevatorConstants {
    constexpr int kLeaderElevatorMotorID = 50;
    constexpr int kFollowerElevatorMotorID = 51;
    
    constexpr int kWristMotorID = 60;  

    constexpr int kWristSensor = 8; 

    constexpr int kScoringMotorID = 61;

    constexpr double kElevatorLevelZero = 0.44;
    constexpr double kelevatorLevelOne = 2.0;
    constexpr double kelevatorLevelTwo = 2.0;
    constexpr double kelevatorLevelThree = 2.0;
    constexpr double kelevatorLevelFour = 25.0;

    constexpr double kelevatorClimbBottom = 25.0;
    constexpr double kelevatorClimbTop = 25.0;

    constexpr double kWristHome = 0.06;
    constexpr double kWristSafe = 2.65;
    constexpr double kWristProcessor = 16.77;
    
    constexpr double kintakeTurns = -6.0;
    constexpr double kdeliveryTurns = -6.0;

}
