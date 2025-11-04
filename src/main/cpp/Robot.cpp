// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandScheduler.h>
#include <cameraserver/CameraServer.h>
#include "subsystems/LimelightHelpers.h"
#include <iostream>

frc::Field2d m_field;
frc::Field2d m_fieldMt1;
constexpr units::time::second_t print_period{500_ms};
units::second_t currentTime = 0_s;


Robot::Robot() {}

void Robot::RobotInit(){
nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("throtle_set",150);
// Do this in either robot or subsystem init
frc::SmartDashboard::PutData("Field", &m_field);
frc::SmartDashboard::PutData("FieldMt1",&m_fieldMt1);


// cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
// camera.SetResolution(640, 480);
// camera.SetFPS(30);

/*
frc::SmartDashboard::PutNumber("Elevator Level Zero", 0.44);
frc::SmartDashboard::PutNumber("Elevator Level One", 8.0);
frc::SmartDashboard::PutNumber("Elevator Level Two", 25.0);
frc::SmartDashboard::PutNumber("Elevator Level Three", 25.0);
frc::SmartDashboard::PutNumber("Elevator Level Four", 25.0);

frc::SmartDashboard::PutNumber("Elevator Climb Bottom", 25.0);
frc::SmartDashboard::PutNumber("Elevator Climb Top", 25.0);

frc::SmartDashboard::PutNumber("Wrist Home", 0.06);
frc::SmartDashboard::PutNumber("Wrist Safe", 2.65);
frc::SmartDashboard::PutNumber("Wrist To Processor", 16.77);

frc::SmartDashboard::PutNumber("Intake Turns Per Second", -6.0);
frc::SmartDashboard::PutNumber("Delivery Turns Per Second", -6.0);

*/

/*
  frc::Preferences::InitDouble("Elevator Level Zero", 0.44);
  frc::Preferences::InitDouble("Elevator Level One", 8.0);
  frc::Preferences::InitDouble("Elevator Level Two", 25.0);
  frc::Preferences::InitDouble("Elevator Level Three", 25.0);
  frc::Preferences::InitDouble("Elevator Level Four", 25.0);

  frc::Preferences::InitDouble("Elevator Climb Bottom", 25.0);
  frc::Preferences::InitDouble("Elevator Climb Top", 25.0);
  
  frc::Preferences::InitDouble("Wrist Home", 0.06);
  frc::Preferences::InitDouble("Wrist Safe", 2.65);
  frc::Preferences::InitDouble("Wrist To Processor", 16.77);

  frc::Preferences::InitDouble("Intake Turns Per Second", -6.0);
  frc::Preferences::InitDouble("Delivery Turns Per Second", -6.0);
  */
}

void Robot::RobotPeriodic() {
  m_container.RobotContainerPeriodic();
  
    frc2::CommandScheduler::GetInstance().Run();
frc::SmartDashboard::PutNumber("LimeLight throtle",nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->GetNumber("throtle_set",200));

  
  
  
  

  // Do this in either robot periodic or subsystem periodic


  frc2::CommandScheduler::GetInstance().Run();


//Not sure if I need this
//LimelightHelpers::setPipelineIndex("limelight", 0);  // Replace 0 with the correct pipeline index

m_field.SetRobotPose(m_container.drivetrain.GetState().Pose);


  if (kUseLimelight) {
    auto const driveState = m_container.drivetrain.GetState();
    auto const heading = driveState.Pose.Rotation().Degrees();
    auto const omega = driveState.Speeds.omega;

    // Set robot orientation for both Limelights
LimelightHelpers::SetRobotOrientation("limelight-front", heading.value(), 0, 0, 0, 0, 0);
LimelightHelpers::SetRobotOrientation("limelight-back", heading.value(), 0, 0, 0, 0, 0);

// Get MegaTag2 pose estimates from both Limelights
auto frontPose = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front");
auto backPose = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back");
auto frontPoseMt1 = LimelightHelpers::getBotPoseEstimate_wpiBlue("limelight-front");
auto backPoseMt1 = LimelightHelpers::getBotPoseEstimate_wpiBlue("limelight-back");

m_fieldMt1.SetRobotPose(frontPoseMt1->pose);

// Choose which one(s) to trust — this example uses both if they’re valid
if (frontPose && frontPose->tagCount > 0 && units::math::abs(omega) < 2_tps) {
    m_container.drivetrain.AddVisionMeasurement(frontPose->pose, frontPose->timestampSeconds);
}

if (backPose && backPose->tagCount > 0 && units::math::abs(omega) < 2_tps) {
    m_container.drivetrain.AddVisionMeasurement(backPose->pose, backPose->timestampSeconds);
}
if (frontPoseMt1 && frontPoseMt1->tagCount > 1 && units::math::abs(omega) < 2_tps) {
    m_container.drivetrain.AddVisionMeasurement(frontPoseMt1->pose, frontPoseMt1->timestampSeconds);
}
if (backPoseMt1 && backPoseMt1->tagCount > 1 && units::math::abs(omega) < 2_tps) {
    m_container.drivetrain.AddVisionMeasurement(backPoseMt1->pose, backPoseMt1->timestampSeconds);
}

  }
  
}

void Robot::DisabledInit() {
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("throtle_set",150);
}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("throtle_set",0);

}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
  nt::NetworkTableInstance::GetDefault().GetTable("limelight-back")->PutNumber("throtle_set",0);
  
}

void Robot::TeleopPeriodic() {


//Testing Preferences smartdashboard
/*
  double elevatorLevelZero = frc::SmartDashboard::GetNumber("Elevator Level Zero", 0.44);
  double elevatorLevelOne = frc::SmartDashboard::GetNumber("Elevator Level One", 8.0);
  double elevatorLevelTwo = frc::SmartDashboard::GetNumber("Elevator Level Two", 25.0);
  double elevatorLevelThree = frc::SmartDashboard::GetNumber("Elevator Level Three", 25.0);
  double elevatorLevelFour = frc::SmartDashboard::GetNumber("Elevator Level Four", 25.0);

  double elevatorClimbReady = frc::SmartDashboard::GetNumber("Elevator Climb Bottom", 25.0);
  double elevatorClimbDesired = frc::SmartDashboard::GetNumber("Elevator Climb Top", 25.0);

  double wristHome = frc::SmartDashboard::GetNumber("Wrist Home", 0.06);
  double wristSafe = frc::SmartDashboard::GetNumber("Wrist Safe", 2.65);
  double wristProcessor = frc::SmartDashboard::GetNumber("Wrist To Processor", 16.77);

  double intakeTurns = frc::SmartDashboard::GetNumber("Intake Turns Per Second", -6.0);
  double deliveryTurns = frc::SmartDashboard::GetNumber("Delivery Turns Per Second", -6.0);
  */
}

void Robot::TeleopExit() {}

  

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

void Robot::CreateDoublePreferenceKey(std::string_view KeyName, double DefaultKeyValue)
  {
    if (!frc::Preferences::ContainsKey(KeyName)) // Check if it doesn't already exit
    {
      frc::Preferences::InitDouble(KeyName, DefaultKeyValue); // Create it and set to value if it doesn't exit
    }
  }

  /**
 * This function creates an Entry in the Preference Table
 * and sets it to a value if it doesn't already exist.
 */
void Robot::CreateStringPreferenceKey(std::string_view KeyName, std::string_view DefaultKeyValue)
{
  if (!frc::Preferences::ContainsKey(KeyName)) // Check if it doesn't already exit
  {
    frc::Preferences::SetString(KeyName, DefaultKeyValue); // Create it and set to value if it doesn't exit
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
