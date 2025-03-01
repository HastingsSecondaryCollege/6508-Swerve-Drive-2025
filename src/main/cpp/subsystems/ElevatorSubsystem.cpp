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
  leadElevatorMotorConfig.Slot0.kP = 1.0;
  //leadElevatorMotorConfig.Slot0.kA = 1.0; 
  //leadElevatorMotorConfig.Slot0.kV = 10.0;
  leadElevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 35.0_rad_per_s_sq;
  leadElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0_rad_per_s;
  
  wristMotorConfig.Slot0.kP = 1.0;
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

//Command Pointers that drive the elevator to set position using above methods
frc2::CommandPtr ElevatorSubsystem::ElevatorLevelZeroCMD() {
  return this->RunOnce([this] {
    ElevatorLevelZero(units::angle::turn_t(frc::Preferences::GetDouble("Elevator Level Zero"))); });
}


frc2::CommandPtr ElevatorSubsystem::ElevatorLevelOneCMD() {
  return this->RunOnce([this] { ElevatorLevelOne(units::angle::turn_t(frc::Preferences::GetDouble("Elevator Level One"))); });
}


frc2::CommandPtr ElevatorSubsystem::ElevatorLevelTwoCMD() {
  return this->RunOnce([this] { ElevatorLevelTwo(units::angle::turn_t(frc::Preferences::GetDouble("Elevator Level Two"))); });
}



//Movement of Wrist methods + CMDs
//Wrist In is scoring/intake position
//Wrist Out is holding position/Scoring balls

void ElevatorSubsystem::WristHome() {
  m_canClimb = false;
  m_wristMotor.SetControl(m_motionMagicControlWrist.WithPosition(0.06_tr)); 
  fmt::println("Just set canClimb false");
}

void ElevatorSubsystem::WristSafe() {
  m_canClimb = true;
  m_wristMotor.SetControl(m_motionMagicControlWrist.WithPosition(2.65_tr));
  fmt::println("Just set canClimb true"); 
}

void ElevatorSubsystem::WristToProcessor() {
  m_canClimb = false;
  m_wristMotor.SetControl(m_motionMagicControlWrist.WithPosition(16.77_tr)); 
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

void ElevatorSubsystem::IntakeCoral(units::voltage::volt_t IntakeVoltage) {
  m_scoringMotor.SetControl(m_percentagePowerCoral.WithOutput(IntakeVoltage)); // -2, further testing
}

void ElevatorSubsystem::DeliverCoral(units::voltage::volt_t DeliveryVoltage) {
  m_scoringMotor.SetControl(m_percentagePowerCoral.WithOutput(DeliveryVoltage)); // -2 for now
}

void ElevatorSubsystem::StopCoralMotor(){
  m_scoringMotor.SetControl(m_percentagePowerCoral.WithOutput(0_V));
}

frc2::CommandPtr ElevatorSubsystem::IntakeCoralCMD() {
  return frc2::FunctionalCommand(
    //Init
    [this]{
      IntakeCoral(units::voltage::volt_t(frc::Preferences::GetDouble("Intake Voltage")));
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
      DeliverCoral(units::voltage::volt_t(frc::Preferences::GetDouble("Delivery Voltage")));
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