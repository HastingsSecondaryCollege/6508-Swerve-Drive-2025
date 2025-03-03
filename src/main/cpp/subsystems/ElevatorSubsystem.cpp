// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"
#include "ctre/phoenix6/controls/MotionMagicVoltage.hpp"
//#include "ctre/phoenix6/configs/MotionMagicConfigs.hpp"
//#include "ctre/phoenix6/configs/TalonFXConfiguration.hpp"
#include <units/angle.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include "Constants.h"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"
#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>







ElevatorSubsystem::ElevatorSubsystem(){

 // Blank configuration object for a TalonFX based motor
  ctre::phoenix6::configs::TalonFXConfiguration leadElevatorMotorConfig{};
  ctre::phoenix6::configs::TalonFXConfiguration wristMotorConfig{};
  

  // Set one of the configuration objects. Refer to Phoenix Tuner X to find wait
  // they are callled
  leadElevatorMotorConfig.Slot0.kP = 10.0;
  //leadElevatorMotorConfig.Slot0.kA = 1.0; 
  //leadElevatorMotorConfig.Slot0.kV = 10.0;
  leadElevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 120.0_rad_per_s_sq;
  leadElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 140.0_rad_per_s;
  
  wristMotorConfig.Slot0.kP = 5.0;
  //wristMotorConfig.Slot0.kA = 1.0; 
  //wristMotorConfig.Slot0.kV = 10.0;
  wristMotorConfig.MotionMagic.MotionMagicAcceleration = 17.0_rad_per_s_sq;
  wristMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 50.0_rad_per_s;


  // Get the Configurator for the motor, then apply the config object that
  // youhave set up.
  m_leadElevatorMotor.GetConfigurator().Apply(leadElevatorMotorConfig);
  m_followElevatorMotor.SetControl(ctre::phoenix6::controls::Follower(ElevatorConstants::kLeaderElevatorMotorID, false));
  
  m_wristMotor.GetConfigurator().Apply(wristMotorConfig);

  //Set neutral on elevators to brake
  m_leadElevatorMotor.SetNeutralMode(1);
  m_followElevatorMotor.SetNeutralMode(1);

}
  // End of ArmSubsystem Constructor

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {
  frc::SmartDashboard::PutBoolean("Can Climb?", m_canClimb);

  //Update Preferences
  m_elevatorLevelZeroHeight = frc::Preferences::GetDouble("Elevator Level Zero");
  m_elevatorLevelOneHeight = frc::Preferences::GetDouble("Elevator Level One");
  m_elevatorLevelTwoHeight = frc::Preferences::GetDouble("Elevator Level Two");
  m_elevatorLevelThreeHeight = frc::Preferences::GetDouble("Elevator Level Three");
  m_elevatorLevelFourHeight = frc::Preferences::GetDouble("Elevator Level Four");

  m_elevatorClimbBottomHeight = frc::Preferences::GetDouble("Elevator Climb Bottom");
  m_elevatorClimbTopHeight = frc::Preferences::GetDouble("Elevator Climb Top");

  m_wristHomePosition = frc::Preferences::GetDouble("Wrist Home");
  m_wristSafePosition = frc::Preferences::GetDouble("Wrist Safe");
  m_wristProcessorPosition = frc::Preferences::GetDouble("Wrist To Processor");

  m_intakeTurns = frc::Preferences::GetDouble("Intake Turns Per Second");
  m_deliveryTurns = frc::Preferences::GetDouble("Delivery Turns Per Second");
}

bool ElevatorSubsystem::IsCoralLoaded() {
  return(m_wristSensor.GetIsDetected().GetValue());
}

// This drives the motor to set turns
void ElevatorSubsystem::ElevatorLevelZero(units::angle::turn_t ElevatorHeight) {
  if (m_canClimb){
  m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(ElevatorHeight));
};}

void ElevatorSubsystem::ElevatorLevelOne(units::angle::turn_t ElevatorHeight) {
  if (m_canClimb){
  m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(ElevatorHeight));
};}

void ElevatorSubsystem::ElevatorLevelTwo(units::angle::turn_t ElevatorHeight) {
  if (m_canClimb){
  m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(ElevatorHeight));
};}

void ElevatorSubsystem::ElevatorLevelThree(units::angle::turn_t ElevatorHeight) {
  if (m_canClimb){
  m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(ElevatorHeight));
};}

void ElevatorSubsystem::ElevatorLevelFour(units::angle::turn_t ElevatorHeight) {
  if (m_canClimb){
  m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(ElevatorHeight));
};}

//Command Pointers that drive the elevator to set position using above methods
frc2::CommandPtr ElevatorSubsystem::ElevatorLevelZeroCMD() {
  return this->RunOnce([this] {
    ElevatorLevelZero(units::angle::turn_t(m_elevatorLevelZeroHeight)); });
}

frc2::CommandPtr ElevatorSubsystem::ElevatorLevelOneCMD() {
  return this->RunOnce([this] { ElevatorLevelOne(units::angle::turn_t(m_elevatorLevelOneHeight)); });
}

frc2::CommandPtr ElevatorSubsystem::ElevatorLevelTwoCMD() {
  return this->RunOnce([this] { ElevatorLevelTwo(units::angle::turn_t(m_elevatorLevelTwoHeight)); });
}

frc2::CommandPtr ElevatorSubsystem::ElevatorLevelThreeCMD() {
  return this->RunOnce([this] { ElevatorLevelTwo(units::angle::turn_t(m_elevatorLevelThreeHeight)); });
}

frc2::CommandPtr ElevatorSubsystem::ElevatorLevelFourCMD() {
  return this->RunOnce([this] { ElevatorLevelTwo(units::angle::turn_t(m_elevatorLevelFourHeight)); });
}



//Movement of Wrist methods + CMDs
//Wrist In is scoring/intake position
//Wrist Out is holding position/Scoring balls

void ElevatorSubsystem::WristHome() {
  m_canClimb = false;
  m_wristMotor.SetControl(m_motionMagicControlWrist.WithPosition(units::angle::turn_t(m_wristHomePosition))); 
  fmt::println("Just set canClimb false");
}

void ElevatorSubsystem::WristSafe() {
  m_canClimb = true;
  m_wristMotor.SetControl(m_motionMagicControlWrist.WithPosition(units::angle::turn_t(m_wristSafePosition)));
  fmt::println("Just set canClimb true"); 
}

void ElevatorSubsystem::WristToProcessor() {
  m_canClimb = false;
  m_wristMotor.SetControl(m_motionMagicControlWrist.WithPosition(units::angle::turn_t(m_wristProcessorPosition))); 
  fmt::println("Just set canClimb false");
}

frc2::CommandPtr ElevatorSubsystem::WristHomeCMD(){
  return this->RunOnce([this] { WristHome();});
}

frc2::CommandPtr ElevatorSubsystem::WristSafeCMD(){
  return this->RunOnce([this] { WristSafe();});
}

frc2::CommandPtr ElevatorSubsystem::WristToProcessorCMD(){
  return this->RunOnce([this] { WristToProcessor();});
}

//Scoring motor direction
// For intaking Algae value should be +0.6_V

void ElevatorSubsystem::IntakeCoral(units::turns_per_second_t IntakeTurns) {
  m_scoringMotor.SetControl(m_turnsPerSecondCoralMotor.WithVelocity(IntakeTurns)); // -2, further testing
}

void ElevatorSubsystem::DeliverCoral(units::turns_per_second_t DeliveryTurns) {
  m_scoringMotor.SetControl(m_turnsPerSecondCoralMotor.WithVelocity(DeliveryTurns)); // -2 for now
}

void ElevatorSubsystem::StopCoralMotor(){
  m_scoringMotor.SetControl(m_turnsPerSecondCoralMotor.WithVelocity(units::angular_velocity::turns_per_second_t(0)));
}

frc2::CommandPtr ElevatorSubsystem::IntakeCoralCMD() {
  return frc2::FunctionalCommand(
    //Init
    [this]{
      IntakeCoral(units::turns_per_second_t(m_intakeTurns));
    },
    //Periodic
    [this]{
    },
    //Command End
    [this](bool interrupted) {
      StopCoralMotor();
    },
    //isFinished
    [this] {return IsCoralLoaded();},
    
    {this}

  ).ToPtr();
}

frc2::CommandPtr ElevatorSubsystem::DeliverCoralCMD() {
  return frc2::FunctionalCommand(
    //Init
    [this]{
      DeliverCoral(units::turns_per_second_t(m_deliveryTurns));
    },
    //Periodic
    [this]{
    },
    //Command End
    [this](bool interrupted) {
      StopCoralMotor();
    },
    //isFinished
    [this]{return !IsCoralLoaded();}

  ).ToPtr();
}

frc2::CommandPtr ElevatorSubsystem::StopCoralMotorCMD() {
  return this->RunOnce([this] {StopCoralMotor();});
}

bool ElevatorSubsystem::CanWeClimb(){
  return m_canClimb;
}