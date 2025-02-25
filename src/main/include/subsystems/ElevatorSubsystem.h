// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>

#include <frc2/command/CommandPtr.h>

#include "Constants.h"

class ElevatorSubsystem : public frc2::SubsystemBase
{
public:
  ElevatorSubsystem();

  void ElevatorLevelZero();
  void ElevatorLevelOne();
  void ElevatorLevelTwo();

  frc2::CommandPtr ElevatorLevelZeroCMD();
  frc2::CommandPtr ElevatorLevelOneCMD();
  frc2::CommandPtr ElevatorLevelTwoCMD();

  bool CanClimb = true;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
 
  ctre::phoenix6::hardware::TalonFX m_leadElevatorMotor{ElevatorConstants::kLeaderElevatorMotorID};
  ctre::phoenix6::hardware::TalonFX m_followElevatorMotor{ElevatorConstants::kFollowerElevatorMotorID};
 

  //Postion Control Objects
 
  ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicControlElevatorLead{0_tr};
  ctre::phoenix6::controls::PositionVoltage m_positionVoltageElevatorLead = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);

  /*  This seems just like the line above it?
  ctre::phoenix6::controls::PositionVoltage m_PositionVoltageControlElevatorLead{0_tr};
*/
};
