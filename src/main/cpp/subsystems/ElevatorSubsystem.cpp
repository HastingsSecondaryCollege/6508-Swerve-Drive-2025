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




ElevatorSubsystem::ElevatorSubsystem(){

 // Blank configuration object for a TalonFX based motor
  ctre::phoenix6::configs::TalonFXConfiguration leadElevatorMotorConfig{};

  // Set one of the configuration objects. Refer to Phoenix Tuner X to find wait
  // they are callled
  leadElevatorMotorConfig.Slot0.kP = 1.0;
  //leadElevatorMotorConfig.Slot0.kA = 1.0; 
  //leadElevatorMotorConfig.Slot0.kV = 10.0;
  leadElevatorMotorConfig.MotionMagic.MotionMagicAcceleration = 10.0_rad_per_s_sq;
  leadElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 50.0_rad_per_s;

  // Get the Configurator for the motor, then apply the config object that
  // youhave set up.
  m_leadElevatorMotor.GetConfigurator().Apply(leadElevatorMotorConfig);

}  // End of ArmSubsystem Constructor

// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {
      if (!CanClimb){
        m_leadElevatorMotor.Set(20.0);
        
    }
}

// This drives the motor to set turns
void ElevatorSubsystem::ElevatorLevelZero() {
  m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(0_tr));
}

void ElevatorSubsystem::ElevatorLevelOne() {
  m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(50_tr));
}

void ElevatorSubsystem::ElevatorLevelTwo() {
  m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(100_tr));
}




frc2::CommandPtr ElevatorSubsystem::ElevatorLevelZeroCMD() {
  return this->RunOnce([this] { 
    ElevatorLevelZero(); 
    });
  }


frc2::CommandPtr ElevatorSubsystem::ElevatorLevelOneCMD() {
  return this->RunOnce([this] { ElevatorLevelOne(); });
}


frc2::CommandPtr ElevatorSubsystem::ElevatorLevelTwoCMD() {
  return this->RunOnce([this] { ElevatorLevelTwo(); });
}