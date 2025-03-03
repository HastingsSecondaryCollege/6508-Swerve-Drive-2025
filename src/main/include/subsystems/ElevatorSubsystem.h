// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

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
  void ElevatorLevelThree(units::angle::turn_t ElevatorHeight);
  void ElevatorLevelFour(units::angle::turn_t ElevatorHeight);

  frc2::CommandPtr ElevatorLevelZeroCMD();
  frc2::CommandPtr ElevatorLevelOneCMD();
  frc2::CommandPtr ElevatorLevelTwoCMD();
  frc2::CommandPtr ElevatorLevelThreeCMD();
  frc2::CommandPtr ElevatorLevelFourCMD();  

  void ElevatorClimbReady(units::angle::turn_t ElevatorHeight);
  void ElevatorClimbDesired(units::angle::turn_t ElevatorHeight);

  frc2::CommandPtr ElevatorClimbReadyCMD();
  frc2::CommandPtr ElevatorClimbDesiredCMD();

  void WristHome();
  void WristSafe();
  void WristToProcessor();
  void WristClimb();

  frc2::CommandPtr WristHomeCMD();
  frc2::CommandPtr WristSafeCMD();
  frc2::CommandPtr WristToProcessorCMD();
  frc2::CommandPtr WristClimbCMD();

  void IntakeCoral(units::voltage::volt_t MotorPower);
  void DeliverCoral(units::voltage::volt_t MotorPower);
  void StopCoralMotor();

  frc2::CommandPtr IntakeCoralCMD();
  frc2::CommandPtr DeliverCoralCMD();
  frc2::CommandPtr StopCoralMotorCMD();

  void IntakeAlgae(units::voltage::volt_t MotorPower);
  void DeliverAlgae(units::voltage::volt_t MotorPower);
  bool HasStalled(double StallCurrent);

  frc2::CommandPtr IntakeAlgaeCMD();
  frc2::CommandPtr DeliverAlgaeCMD();

  bool m_canClimb = false; //set canClimb to false by default
  bool CanWeClimb();

  bool IsCoralLoaded();

  bool m_isScoringHeight = false;
  bool GetScoringHeight();

  double  m_elevatorLevelZeroHeight;
  double  m_elevatorLevelOneHeight;
  double  m_elevatorLevelTwoHeight;
  double  m_elevatorLevelThreeHeight;
  double  m_elevatorLevelFourHeight;

  double  m_elevatorClimbReadyHeight;
  double  m_elevatorClimbDesiredHeight;
  
  double m_wristHomePosition;
  double m_wristSafePosition;
  double m_wristProcessorPosition;
  double m_wristClimbPosition;

  double m_intakeCoralTurns;
  double m_deliveryCoralTurns;

  double m_intakeAlgaeTurns;
  double m_deliveryAlgaeTurns;

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

  ctre::phoenix6::controls::NeutralOut m_brake{};

  ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicControlElevatorLead{0_tr};
  ctre::phoenix6::controls::PositionVoltage m_positionVoltageElevatorLead = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);
 
  ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicControlWrist{0_tr};
  ctre::phoenix6::controls::PositionVoltage m_positionVoltageWrist = ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0);

  //ctre::phoenix6::controls::VelocityVoltage m_turnsPerSecondCoralMotor = ctre::phoenix6::controls::VelocityVoltage {0_tps}.WithSlot(0);
  ctre::phoenix6::controls::VoltageOut m_percentagePowerCoral{0_V};

};
