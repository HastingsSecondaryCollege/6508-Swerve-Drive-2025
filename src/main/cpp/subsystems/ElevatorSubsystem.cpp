// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ElevatorSubsystem.h"





ElevatorSubsystem::ElevatorSubsystem(){

    //controls::DutyCycleOut m_request[0]

    ctre::phoenix6::configs::TalonFXConfiguration leadElevatorMotorConfig{};
    ctre::phoenix6::configs::TalonFXConfiguration followElevatorMotorConfig{};    
    
    leadElevatorMotorConfig.Slot0.kP  = 500.0;
    followElevatorMotorConfig.Slot0.kP = 500.0;

    leadElevatorMotorConfig.Slot0.kV  = 75.0;
    followElevatorMotorConfig.Slot0.kV = 75.0;

 
    //leftClimbMotorConfig.Feedback.RotorToSensorRatio  = 640;

    
    //leftClimbMotorConfig.Feedback.FeedbackSensorSource  = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;
    //rightClimbMotorConfig.Feedback.FeedbackSensorSource = ctre::phoenix6::signals::FeedbackSensorSourceValue::RemoteCANcoder;

    //leftClimbMotorConfig.Feedback.SensorToMechanismRatio  = 1.0;
    //rightClimbMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
 
   

    leadElevatorMotorConfig.MotionMagic.MotionMagicAcceleration  = 1_tr_per_s_sq; //0.14
    //followElevatorMotorConfig.Slot0.MotionMagic.MotionMagicAcceleration = 0.50_tr_per_s_sq;
    followElevatorMotorConfig.MotionMagic.MotionMagicAcceleration  = 0.14_tr_per_s_sq; //0.14


    leadElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity  = 1_tps;  //0.14
   //followElevatorMotorConfig.Slot0.kP.MotionMagic.MotionMagicCruiseVelocity = 0.145_tps;
    followElevatorMotorConfig.MotionMagic.MotionMagicCruiseVelocity  = 0.145_tps;  //0.14

    //leadElevatorMotorConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    //followElevatorMotorConfig.Slot0.kP.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
    //followElevatorMotorConfig.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;

    m_leadElevatorMotor.GetConfigurator().Apply(leadElevatorMotorConfig);
    m_followElevatorMotor.GetConfigurator().Apply(followElevatorMotorConfig);  

    //m_followElevatorMotor.SetControl(controls::Follower{m_leadElevatorMotor.GetDeviceId(), false});


}
// This method will be called once per scheduler run
void ElevatorSubsystem::Periodic() {
    //if (!CanClimb){
        m_leadElevatorMotor.Set(20.0);
   // }
}
/*
void ElevatorSubsystem::ElevatorLevelZero(){
    if (CanClimb){
        m_leadElevatorMotor.SetControl(m_motionMagicControlElevatorLead.WithPosition(50_tr));//0.1_tr
    }
}

frc2::CommandPtr ElevatorSubsystem::ElevatorLevelZeroCMD(){
    return this->RunOnce([this] { ElevatorLevelZero(); });
    
}
*/