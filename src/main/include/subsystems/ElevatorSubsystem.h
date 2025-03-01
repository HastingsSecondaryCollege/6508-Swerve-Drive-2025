// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/CANrange.hpp>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/FunctionalCommand.h>

#include "Constants.h"

class ElevatorSubsystem : public frc2::SubsystemBase
{
public:
  ElevatorSubsystem();

  void ElevatorLevelZero(units::angle::turn_t ElevatorHeight);
  void ElevatorLevelOne(units::angle::turn_t ElevatorHeight);
  void ElevatorLevelTwo(units::angle::turn_t ElevatorHeight);

  frc2::CommandPtr ElevatorLevelZeroCMD();
  frc2::CommandPtr ElevatorLevelOneCMD();
  frc2::CommandPtr ElevatorLevelTwoCMD();
  
  void WristHome();
  void WristSafe();
  void WristToProcessor();

  frc2::CommandPtr WristHomeCMD();
  frc2::CommandPtr WristSafeCMD();
  frc2::CommandPtr WristToProcessorCMD();

  void IntakeCoral(units::voltage::volt_t IntakeVoltage);
  void DeliverCoral();
  void StopCoralMotor();

  frc2::CommandPtr IntakeCoralCMD();
  frc2::CommandPtr DeliverCoralCMD();
  frc2::CommandPtr StopCoralMotorCMD();


  bool m_canClimb = false; //set canClimb to false by default
  bool CanWeClimb();

  bool IsCoralLoaded();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
 
  ctre::phoenix6::hardware::TalonFX m_leadElevatorMotor{ElevatorConstants::kLeaderElevatorMotorID};
  ctre::phoenix6::hardware::TalonFX m_followElevatorMotor{ElevatorConstants::kFollowerElevatorMotorID};

  ctre::phoenix6::hardware::TalonFX m_wristMotor{ElevatorConstants::kWristMotorID};
  ctre::phoenix6::hardware::CANrange m_wristSensor{ElevatorConstants::kWristSensor};
  ctre::phoenix6::hardware::TalonFX m_scoringMotor{ElevatorConstants::kScoringMotorID};

  //Postion Control Objects
 
  ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicControlElevatorLead{0_tr};
  ctre::phoenix6::controls::PositionVoltage m_positionVoltageElevatorLead = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);
 
  ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicControlWrist{0_tr};
  ctre::phoenix6::controls::PositionVoltage m_positionVoltageWrist = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);

  ctre::phoenix6::controls::VoltageOut m_percentagePowerCoral{0_V};



};
