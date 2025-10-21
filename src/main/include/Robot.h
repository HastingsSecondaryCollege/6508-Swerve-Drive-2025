// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>
#include <frc/Preferences.h>
#include <frc/Timer.h>
#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 
 private:
  

 public:
  Robot();
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DisabledExit() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void TestExit() override;

  void CreateDoublePreferenceKey(std::string_view KeyName, double DefaultKeyValue);
  void CreateStringPreferenceKey(std::string_view KeyName, std::string_view DefaultKeyValue);

 private:
  //std::optional<frc2::CommandPtr> m_autonomousCommand;
 frc2::Command *m_autonomousCommand;
  RobotContainer m_container;
   static constexpr bool kUseLimelight = true;
};
