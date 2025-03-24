// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"
#include "ctre/phoenix6/controls/MotionMagicVoltage.hpp"
// #include "ctre/phoenix6/configs/MotionMagicConfigs.hpp"
// #include "ctre/phoenix6/configs/TalonFXConfiguration.hpp"
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"
#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "generated/TunerConstants.h"

ElevatorSubsystem::ElevatorSubsystem()
{

  // Blank configuration object for a TalonFX based motor
  ctre::phoenix6::configs::TalonFXConfiguration leadElevatorMotorConfig{};
  ctre::phoenix6::configs::TalonFXConfiguration wristMotorConfig{};
  ctre::phoenix6::configs::TalonFXConfiguration coralMotorConfig{};

  // Set one of the configuration objects. Refer to Phoenix Tuner X to find wait
  // they are callled
  leadElevatorMotorConfig.Slot0.kP = 10.0;
  //leadElevatorMotorConfig.Slot0.kA = 0.1;
  //leadElevatorMotorConfig.Slot0.kV = 0.12;
  leadElevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 200.0_rad_per_s_sq;
  leadElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 200.0_rad_per_s;
  leadElevatorMotorConfig.MotionMagic.MotionMagicJerk = 2000_rad_per_s_cu;

  wristMotorConfig.Slot0.kP = 10.0;
   wristMotorConfig.Slot0.kA = 0.1;
   wristMotorConfig.Slot0.kV = 0.12;
  wristMotorConfig.MotionMagic.MotionMagicAcceleration = 80.0_rad_per_s_sq;
  wristMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 80.0_rad_per_s;
  wristMotorConfig.MotionMagic.MotionMagicJerk = 600_rad_per_s_cu;

  coralMotorConfig.Slot0.kS = 0.1;  // To account for friction, add 0.1 V of static feedforward
  coralMotorConfig.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
  coralMotorConfig.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
  coralMotorConfig.Slot0.kI = 0;    // No output for integrated error
  coralMotorConfig.Slot0.kD = 0;    // No output for error derivative
  // Peak output of 8 volts
  coralMotorConfig.Voltage.PeakForwardVoltage = 8_V;
  coralMotorConfig.Voltage.PeakReverseVoltage = -8_V;

  // Get the Configurator for the motor, then apply the config object that
  // youhave set up.
  m_leadElevatorMotor.GetConfigurator().Apply(leadElevatorMotorConfig);
  m_followElevatorMotor.SetControl(ctre::phoenix6::controls::Follower(ElevatorConstants::kLeaderElevatorMotorID, false));

  m_wristMotor.GetConfigurator().Apply(wristMotorConfig);

  m_scoringMotor.GetConfigurator().Apply(coralMotorConfig);

  // Set neutral on elevators to brake
  m_scoringMotor.SetNeutralMode(true);
  m_leadElevatorMotor.SetNeutralMode(true);
}
// End of ArmSubsystem Constructor

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic()
{
  frc::SmartDashboard::PutBoolean("Can Climb?", m_canClimb);
  using namespace ElevatorConstants;
  // Update Preferences
  m_elevatorLevelZeroHeight = kElevatorLevelZero;
  m_elevatorLevelOneHeight = kElevatorLevelOne;
  m_elevatorLevelTwoHeight = kElevatorLevelTwo;
  m_elevatorLevelThreeHeight = kElevatorLevelThree;
  m_elevatorLevelFourHeight = kElevatorLevelFour;
  m_elevatorLevelAlgaeRemoveLow = kElevatorLevelAlgaeRemoveLow;
  m_elevatorLevelAlgaeRemoveHigh = kElevatorLevelAlgaeRemoveHigh;

  m_elevatorClimbReadyHeight = kElevatorClimbReady;
  m_elevatorClimbDesiredHeight = kElevatorClimbDesired;

  m_wristHomePosition = kWristHome;
  m_wristSafePosition = kWristSafe;
  m_wristAlgaeRemovePosition = kWristAlgaeRemove;
  m_wristProcessorPosition = kWristProcessor;
  m_wristClimbPosition = kWristClimb;
  m_wristDeliverHighPosition = kWristDeliverHigh;

  m_intakeCoralTurns = kIntakeCoralTurns;
  m_deliveryCoralLowTurns = kDeliveryCoralLowTurns;
  m_deliveryCoralMiddleTurns = kDeliveryCoralMiddleTurns;
  m_deliveryCoralHighTurns = kDeliveryCoralHighTurns;

  m_intakeAlgaeTurns = kIntakeAlgaeVolts;
  m_deliveryAlgaeTurns = kDeliveryAlgaeVolts;

  m_getIntakePosition = m_scoringMotor.GetPosition().GetValueAsDouble();
}
/**/
double ElevatorSubsystem::IntakePositionPlusThree()
{
  return (m_getIntakePosition + 3.0);
  // return (m_getIntakePosition() + 3.0);

  // return (m_scoringMotor.GetPosition().GetValueAsDouble() + 3.0);
}

bool ElevatorSubsystem::IsCoralLoaded()
{
  return (m_wristSensor.GetIsDetected().GetValue());
}

// This drives the motor to set turns
void ElevatorSubsystem::ElevatorLevelZero(units::angle::turn_t ElevatorHeight)
{
  if (m_canClimb)
  {
    m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(ElevatorHeight));
    m_isScoringHeight = false;
    fmt::println("Just finished Elevator Level Zero");
  }
  else
  {
    fmt::println("Cannot Climb");
  };
}

void ElevatorSubsystem::ElevatorLevelOne(units::angle::turn_t ElevatorHeight)
{
  if (m_canClimb)
  {
    m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(ElevatorHeight));
    m_isScoringHeight = false;
    fmt::println("Just finished Elevator Level One");
  }
  else
  {
    fmt::println("Cannot Climb");
  };
}

void ElevatorSubsystem::ElevatorLevelTwo(units::angle::turn_t ElevatorHeight)
{
  if (m_canClimb)
  {
    m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(ElevatorHeight));
    m_isScoringHeight = false;
    fmt::println("Just finished Elevator Level Two");
  }
  else
  {
    fmt::println("Cannot Climb");
  };
}

void ElevatorSubsystem::ElevatorLevelThree(units::angle::turn_t ElevatorHeight)
{
  if (m_canClimb)
  {
    m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(ElevatorHeight));
    m_isScoringHeight = false;
    fmt::println("Just finished Elevator Level Three");
  }
  else
  {
    fmt::println("Cannot Climb");
  };
}

void ElevatorSubsystem::ElevatorLevelFour(units::angle::turn_t ElevatorHeight)
{
  if (m_canClimb)
  {
    m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(ElevatorHeight));
    m_isScoringHeight = true;
    fmt::println("Just finished Elevator Level Zero");
  }
  else
  {
    fmt::println("Cannot Climb");
  };
}

void ElevatorSubsystem::ElevatorLevelAlgaeRemoveLow(units::angle::turn_t ElevatorHeight)
{
  if (m_canClimb)
  {
    m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(ElevatorHeight));
    m_isScoringHeight = false;
    fmt::println("Just finished Elevator Level Two");
  }
  else
  {
    fmt::println("Cannot Climb");
  };
}

void ElevatorSubsystem::ElevatorLevelAlgaeRemoveHigh(units::angle::turn_t ElevatorHeight)
{
  if (m_canClimb)
  {
    m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(ElevatorHeight));
    m_isScoringHeight = false;
    fmt::println("Just finished Elevator Level Two");
  }
  else
  {
    fmt::println("Cannot Climb");
  };
}

// Command Pointers that drive the elevator to set position using above methods
frc2::CommandPtr ElevatorSubsystem::ElevatorLevelZeroCMD()
{
  return this->RunOnce([this]
                       { ElevatorLevelZero(units::angle::turn_t(m_elevatorLevelZeroHeight)); });
}

frc2::CommandPtr ElevatorSubsystem::ElevatorLevelOneCMD()
{
  return this->RunOnce([this]
                       { ElevatorLevelOne(units::angle::turn_t(m_elevatorLevelOneHeight)); });
}

frc2::CommandPtr ElevatorSubsystem::ElevatorLevelTwoCMD()
{
  return this->RunOnce([this]
                       { ElevatorLevelTwo(units::angle::turn_t(m_elevatorLevelTwoHeight)); });
}

frc2::CommandPtr ElevatorSubsystem::ElevatorLevelThreeCMD()
{
  return this->RunOnce([this]
                       { ElevatorLevelTwo(units::angle::turn_t(m_elevatorLevelThreeHeight)); });
}

frc2::CommandPtr ElevatorSubsystem::ElevatorLevelFourCMD()
{
  return this->RunOnce([this]
                       { ElevatorLevelTwo(units::angle::turn_t(m_elevatorLevelFourHeight)); });
}

frc2::CommandPtr ElevatorSubsystem::ElevatorLevelAlgaeRemoveLowCMD()
{
  return this->RunOnce([this]
                       { ElevatorLevelAlgaeRemoveLow(units::angle::turn_t(m_elevatorLevelAlgaeRemoveLow)); });
}

frc2::CommandPtr ElevatorSubsystem::ElevatorLevelAlgaeRemoveHighCMD()
{
  return this->RunOnce([this]
                       { ElevatorLevelAlgaeRemoveHigh(units::angle::turn_t(m_elevatorLevelAlgaeRemoveHigh)); });
}

void ElevatorSubsystem::ElevatorClimbReady(units::angle::turn_t ElevatorHeight)
{
  if (m_canClimb)
  {
    m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(ElevatorHeight));
    fmt::println("Just finished Elevator Climb To Bottom");
  }
  else
  {
    fmt::println("Cannot Climb");
  };
}

void ElevatorSubsystem::ElevatorClimbDesired(units::angle::turn_t ElevatorHeight)

  {
    m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(ElevatorHeight));
    fmt::println("Just finished Elevator Climb To Top");
  
}

frc2::CommandPtr ElevatorSubsystem::ElevatorClimbReadyCMD()
{
  return this->RunOnce([this]
                       { ElevatorClimbReady(units::angle::turn_t(m_elevatorClimbReadyHeight)); });
}

frc2::CommandPtr ElevatorSubsystem::ElevatorClimbDesiredCMD()
{
  return this->RunOnce([this]
                       { ElevatorClimbDesired(units::angle::turn_t(m_elevatorClimbDesiredHeight)); });
}

frc2::CommandPtr ElevatorSubsystem::ReconfigMotionMagicCMD()
{
  return this->RunOnce([this]
                       {
// Blank configuration object for a TalonFX based motor
  ctre::phoenix6::configs::TalonFXConfiguration leadElevatorMotorConfig{};
 leadElevatorMotorConfig.Slot0.kP = 10.0;
  leadElevatorMotorConfig.Slot0.kA = 0.10; 
  leadElevatorMotorConfig.Slot0.kV = 0.12;
  leadElevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 50.0_rad_per_s_sq;
  leadElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 80.0_rad_per_s;
   m_leadElevatorMotor.GetConfigurator().Apply(leadElevatorMotorConfig); });
}




// Movement of Wrist methods + CMDs
// Wrist In is scoring/intake position
// Wrist Out is holding position/Scoring balls

void ElevatorSubsystem::WristHome()
{
  m_canClimb = false;
  m_wristMotor.SetControl(m_motionMagicControlWrist.WithPosition(units::angle::turn_t(m_wristHomePosition)));
  fmt::println("Just set canClimb false");
  fmt::println("Just finished WristHome");
}

void ElevatorSubsystem::WristSafe()
{
  m_canClimb = true;
  m_wristMotor.SetControl(m_motionMagicControlWrist.WithPosition(units::angle::turn_t(m_wristSafePosition)));
  fmt::println("Just set canClimb true");
  fmt::println("Just finished WristSafe");
}

void ElevatorSubsystem::WristAlgaeRemove()
{
  m_canClimb = true;
  m_wristMotor.SetControl(m_motionMagicControlWrist.WithPosition(units::angle::turn_t(m_wristAlgaeRemovePosition)));
  fmt::println("Just set canClimb true");
  fmt::println("Just finished WristAlgeaRemove");
}

void ElevatorSubsystem::WristToProcessor()
{
  m_canClimb = false;
  m_wristMotor.SetControl(m_motionMagicControlWrist.WithPosition(units::angle::turn_t(m_wristProcessorPosition)));
  fmt::println("Just set canClimb false");
  fmt::println("Just finished WristToProcessor");
}

void ElevatorSubsystem::WristClimb()
{
  m_canClimb = false;
  m_wristMotor.SetControl(m_motionMagicControlWrist.WithPosition(units::angle::turn_t(m_wristClimbPosition)));
}

void ElevatorSubsystem::WristDeveliverHigh()
{
  m_canClimb = true;
  m_wristMotor.SetControl(m_motionMagicControlWrist.WithPosition(units::angle::turn_t(m_wristDeliverHighPosition)));
  fmt::println("Just set canClimb true");
  fmt::println("Just finished WristDeveliverHigh");
}

frc2::CommandPtr ElevatorSubsystem::WristHomeCMD()
{
  return this->RunOnce([this]
                       { WristHome(); });
}

frc2::CommandPtr ElevatorSubsystem::WristSafeCMD()
{
  return this->RunOnce([this]
                       { WristSafe(); });
}

frc2::CommandPtr ElevatorSubsystem::WristAlgaeRemoveCMD()
{
  return this->RunOnce([this]
                       { WristAlgaeRemove(); });
}
frc2::CommandPtr ElevatorSubsystem::WristToProcessorCMD()
{
  return this->RunOnce([this]
                       { WristToProcessor(); });
}

frc2::CommandPtr ElevatorSubsystem::WristClimbCMD()
{
  return this->RunOnce([this]
                       { WristClimb(); });
}

frc2::CommandPtr ElevatorSubsystem::WristDeveliverHighCMD()
{
  return this->RunOnce([this]
                       { WristDeveliverHigh(); });
}
// Scoring motor direction
//  For intaking Algae value should be +0.6_V

void ElevatorSubsystem::IntakeCoral(units::voltage::volt_t MotorPower)
{
  m_scoringMotor.SetControl(m_percentagePowerCoral.WithOutput(MotorPower)); // -2, further testing
}

void ElevatorSubsystem::DeliverCoralLow(units::voltage::volt_t MotorPower)
{
  m_scoringMotor.SetControl(m_percentagePowerCoral.WithOutput(MotorPower)); // -2
}

void ElevatorSubsystem::DeliverCoralMiddle(units::voltage::volt_t MotorPower)
{
  m_scoringMotor.SetControl(m_percentagePowerCoral.WithOutput(MotorPower)); // -2
}

void ElevatorSubsystem::DeliverCoralHigh(units::voltage::volt_t MotorPower)
{
  m_scoringMotor.SetControl(m_percentagePowerCoral.WithOutput(MotorPower)); // -2
}

void ElevatorSubsystem::StopCoralMotor()
{
  m_scoringMotor.SetControl(m_percentagePowerCoral.WithOutput(0_V));
}

void ElevatorSubsystem::SetIntakePosition(double IntakePosition)
{
  m_scoringMotor.SetControl(m_positionVoltageCoral.WithPosition(units::angle::turn_t(IntakePosition)));
}

frc2::CommandPtr ElevatorSubsystem::StopCoralMotorCMD()
{
  return this->RunOnce([this]
                       { StopCoralMotor(); });
}

frc2::CommandPtr ElevatorSubsystem::IntakeCoralCMD()
{
  return frc2::FunctionalCommand(
             // Init
             [this]
             {
               IntakeCoral(units::voltage::volt_t(m_intakeCoralTurns));
             },
             // Periodic
             [this] {},
             // Command End
             [this](bool interrupted)
             {
               SetIntakePosition(IntakePositionPlusThree());
               StopCoralMotor();
               fmt::println("Just finished Intake Coral CMD");
             },
             // isFinished
             [this]
             { return IsCoralLoaded(); },

             {this}

             )
      .ToPtr();
}

frc2::CommandPtr ElevatorSubsystem::DeliverCoralMiddleCMD()
{
  return frc2::FunctionalCommand(
             // Init
             [this]
             {
               DeliverCoralMiddle(units::voltage::volt_t(m_deliveryCoralMiddleTurns));
             },
             // Periodic
             [this] {},
             // Command End
             [this](bool interrupted)
             {
               StopCoralMotor();
               fmt::println("Just finished Deliver Coral CMD");
             },
             // isFinished
             [this]
             { return !IsCoralLoaded(); }

             )
      .ToPtr();
}

void ElevatorSubsystem::IntakeAlgae(units::voltage::volt_t MotorPower)
{
  m_scoringMotor.SetControl(m_percentagePowerCoral.WithOutput(MotorPower));
}

void ElevatorSubsystem::DeliverAlgae(units::voltage::volt_t MotorPower)
{
  m_scoringMotor.SetControl(m_percentagePowerCoral.WithOutput(MotorPower));
}

bool ElevatorSubsystem::HasStalled(double StallCurrent)
{
  if (fabs(m_scoringMotor.GetSupplyCurrent().GetValueAsDouble()) > StallCurrent)
  {
    return true;
  }
  else
  {
    return false;
  }
}

frc2::CommandPtr ElevatorSubsystem::IntakeAlgaeCMD()
{
  return frc2::FunctionalCommand(
             // Init
             [this]
             {
               DeliverAlgae(units::voltage::volt_t(m_intakeAlgaeTurns));
             },
             // Periodic
             [this] {},
             // Command End
             [this](bool interrupted)
             {
               StopCoralMotor();
               fmt::println("Just finished Intake Algae CMD");
             },
             // isFinished
             [this]
             { return HasStalled(20); },

             {this}

             )
      .ToPtr();
}

frc2::CommandPtr ElevatorSubsystem::DeliverAlgaeCMD()
{
  return frc2::FunctionalCommand(
             // Init
             [this]
             {
               IntakeCoral(units::voltage::volt_t(m_deliveryAlgaeTurns));
             },
             // Periodic
             [this] {},
             // Command End
             [this](bool interrupted)
             {
               StopCoralMotor();
               fmt::println("Just finished Deliver Algae CMD");
             },
             // isFinished
             [this]
             { return HasStalled(20); },

             {this}

             )
      .ToPtr().WithTimeout(1_s);
}

bool ElevatorSubsystem::CanWeClimb()
{
  return m_canClimb;
}

bool ElevatorSubsystem::GetScoringHeight()
{
  return m_isScoringHeight;
}
