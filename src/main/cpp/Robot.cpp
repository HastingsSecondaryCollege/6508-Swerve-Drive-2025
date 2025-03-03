// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
 
#include <frc2/command/CommandScheduler.h>
#include <cameraserver/CameraServer.h>
Robot::Robot() {}

void Robot::RobotInit(){

cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
camera.SetResolution(640, 480);
camera.SetFPS(30);

  frc::Preferences::InitDouble("Elevator Level Zero", 0.0);
  frc::Preferences::InitDouble("Elevator Level One", 8.0);
  frc::Preferences::InitDouble("Elevator Level Two", 25.0);
  frc::Preferences::InitDouble("Elevator Level Three", 25.0);
  frc::Preferences::InitDouble("Elevator Level Four", 25.0);

  frc::Preferences::InitDouble("Elevator Climb Bottom", 25.0);
  frc::Preferences::InitDouble("Elevator Climb Top", 25.0);
  
  frc::Preferences::InitDouble("Wrist Home", 0.06);
  frc::Preferences::InitDouble("Wrist Safe", 2.65);
  frc::Preferences::InitDouble("Wrist To Processor", 16.77);

  frc::Preferences::InitDouble("Intake Turns Per Second", -2.0);
  frc::Preferences::InitDouble("Delivery Turns Per Second", -2.0);
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {}

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
