// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Commands.h>
#include "subsystems/ElevatorSubsystem.h"
#include <frc2/command/InstantCommand.h>



RobotContainer::RobotContainer() {
    ConfigureButtonBindings();
    
    drivetrain.SetDefaultCommand(
        drivetrain.ApplyRequest([this]() -> auto&& {
            return drive.WithVelocityX(
                        ApplyDeadbandSquaredInputs(-m_stick.GetY(), 0.1) * 
                        ((1 - m_stick.GetThrottle()) / 2) * MaxSpeed) // Forward/backward
                .WithVelocityY(
                        ApplyDeadbandSquaredInputs(-m_stick.GetX(), 0.1) * 
                        ((1 - m_stick.GetThrottle()) / 2) * MaxSpeed) // Left/right
                .WithRotationalRate(
                        ApplyDeadbandSquaredInputs(-m_stick.GetZ(), 0.15) * MaxAngularRate); // Rotation
        })
    );
    
}

// Apply a deadband to joystick input, ensuring small inputs are ignored
double RobotContainer::ApplyDeadband(double joystickValue, double deadband) {
    if (fabs(joystickValue) > deadband) {
        return (joystickValue > 0 ? 1 : -1) * ((fabs(joystickValue) - deadband) / (1.0 - deadband));
    }
    return 0.0;
}

// Apply deadband and square input values for smoother control
double RobotContainer::ApplyDeadbandSquaredInputs(double joystickValue, double deadband) {
    double adjustedValue = ApplyDeadband(joystickValue, deadband);
    return adjustedValue * fabs(adjustedValue); // Squaring maintains direction
}

// ADD THE FUNCTION HERE 👇👇👇
void RobotContainer::ConfigureButtonBindings() {
    // Example joystick button bindings
    frc2::JoystickButton(&m_stick, 1).OnTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    frc2::JoystickButton(&m_stick, 2).OnTrue(drivetrain.ApplyRequest([this]() -> auto&& {
        return point.WithModuleDirection(frc::Rotation2d{-m_stick.GetY(), -m_stick.GetX()});
    }));

  frc2::JoystickButton(&m_stick, 8).OnTrue(m_elevatorSubsystem.ElevatorLevelZeroCMD());
  frc2::JoystickButton(&m_stick, 10).OnTrue(m_elevatorSubsystem.ElevatorLevelOneCMD());
  frc2::JoystickButton(&m_stick, 12).OnTrue(m_elevatorSubsystem.ElevatorLevelTwoCMD());

    // Add additional bindings here as needed
}

// Autonomous command placeholder
frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    return frc2::cmd::Print("No autonomous command configured");
}
