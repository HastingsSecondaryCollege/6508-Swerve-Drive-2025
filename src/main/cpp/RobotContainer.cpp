// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Commands.h>
#include "subsystems/ElevatorSubsystem.h"
#include <frc2/command/InstantCommand.h>
#include <frc/Preferences.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/POVButton.h>
#include <pathplanner/lib/auto/AutoBuilder.h>

//PathPlanner
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/path/PathPlannerPath.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <memory>

#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/RunCommand.h>
#include <units/angle.h>
#include <units/velocity.h>

using namespace pathplanner;

RobotContainer::RobotContainer() {

  NamedCommands::registerCommand("Elevator Zero",m_elevatorSubsystem.ElevatorLevelZeroCMD());
  NamedCommands::registerCommand("Elevator One",m_elevatorSubsystem.ElevatorLevelOneCMD());
  NamedCommands::registerCommand("Elevator Two",m_elevatorSubsystem.ElevatorLevelTwoCMD());
  NamedCommands::registerCommand("Elevator Three",m_elevatorSubsystem.ElevatorLevelThreeCMD());
  NamedCommands::registerCommand("Elevator Four",m_elevatorSubsystem.ElevatorLevelFourCMD());
  NamedCommands::registerCommand("Elevator Algae Remove Low",m_elevatorSubsystem.ElevatorLevelAlgaeRemoveLowCMD());
  NamedCommands::registerCommand("Elevator Algae Remove High",m_elevatorSubsystem.ElevatorLevelAlgaeRemoveHighCMD());

  NamedCommands::registerCommand("Wrist Safe",m_elevatorSubsystem.WristSafeCMD());
  NamedCommands::registerCommand("Wrist Home",m_elevatorSubsystem.WristHomeCMD());
  NamedCommands::registerCommand("Wrist Processor",m_elevatorSubsystem.WristToProcessorCMD());
  NamedCommands::registerCommand("Wrist Algae Remove",m_elevatorSubsystem.WristAlgaeRemoveCMD());
  NamedCommands::registerCommand("Wrist Deliver High",m_elevatorSubsystem.WristDeveliverHighCMD());

  NamedCommands::registerCommand("Intake Coral",m_elevatorSubsystem.IntakeCoralCMD());
  NamedCommands::registerCommand("Deliver Coral",m_elevatorSubsystem.DeliverCoralMiddleCMD());
  


autoChooser = pathplanner::AutoBuilder::buildAutoChooser("Tests");
    frc::SmartDashboard::PutData("Auto Mode", &autoChooser);

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
    /*
    frc2::JoystickButton(&m_stick, 1).OnTrue(drivetrain.ApplyRequest([this]() -> auto&& { return brake; }));
    
    */
   // frc2::JoystickButton(&m_stick, 11).OnTrue(drivetrain.ApplyRequest([this]() -> auto&& {
   //     return point.WithModuleDirection(frc::Rotation2d{-m_stick.GetY(), -m_stick.GetX()});
   // }));
    //

    //Reset Robot Pose
    //frc2::JoystickButton(&m_stick, 11).OnTrue(drivetrain.RunOnce([this] { drivetrain.SeedFieldCentric(); }));
    //drivetrain.RegisterTelemetry([this](auto const &state) { logger.Telemeterize(state); });


    //Intake Coral
  frc2::JoystickButton(&m_stick, 1).OnTrue(
    frc2::cmd::Sequence(  
        m_elevatorSubsystem.WristSafeCMD(),
        m_elevatorSubsystem.ElevatorLevelZeroCMD(),
        m_elevatorSubsystem.WristHomeCMD(),
        m_elevatorSubsystem.IntakeCoralCMD(),
        //frc2::cmd::Wait(1.0_s),
        m_elevatorSubsystem.WristSafeCMD()
    )
  );
  
    //Deliver Coral
  frc2::JoystickButton(&m_stick, 2).OnTrue(
    frc2::cmd::Sequence(
    m_elevatorSubsystem.DeliverCoralMiddleCMD(),
    m_elevatorSubsystem.WristSafeCMD(),
    m_elevatorSubsystem.ElevatorLevelZeroCMD()
    
    ));

    //Process Algae Position
  frc2::JoystickButton(&m_stick, 4).OnTrue(
    frc2::cmd::Sequence(
      //m_elevatorSubsystem.WristSafeCMD(),
      m_elevatorSubsystem.ElevatorLevelZeroCMD(),
      m_elevatorSubsystem.WristToProcessorCMD()
    )
    );
    

  //Deliver Algae
  frc2::JoystickButton(&m_stick, 6).OnTrue(
    frc2::cmd::Sequence(
     m_elevatorSubsystem.DeliverAlgaeCMD(),
     frc2::cmd::Wait(2.0_s),
     m_elevatorSubsystem.WristSafeCMD()
    )
  );

 //Intake Algae Level 1
  frc2::JoystickButton(&m_stick, 3).OnTrue(    
    frc2::cmd::Sequence(
        m_elevatorSubsystem.WristAlgaeRemoveCMD(),
        m_elevatorSubsystem.ElevatorLevelAlgaeRemoveLowCMD(),
        m_elevatorSubsystem.IntakeAlgaeCMD(),
        m_elevatorSubsystem.ElevatorLevelZeroCMD()
    ));

    //Intake Algae Level 2
  frc2::JoystickButton(&m_stick, 5).OnTrue(    
    frc2::cmd::Sequence(
        m_elevatorSubsystem.WristAlgaeRemoveCMD(),
        m_elevatorSubsystem.ElevatorLevelAlgaeRemoveHighCMD(),
        m_elevatorSubsystem.IntakeAlgaeCMD(),
         m_elevatorSubsystem.ElevatorLevelZeroCMD()
    ));

    
/*
    //Ready Climb
  frc2::JoystickButton(&m_stick, 4).OnTrue(
    frc2::cmd::Sequence(
      m_elevatorSubsystem.WristClimbCMD(),
      m_elevatorSubsystem.ElevatorClimbReadyCMD()
    ));

    //Complete Climb
  frc2::JoystickButton(&m_stick, 6).OnTrue(
    frc2::cmd::Sequence(
      m_elevatorSubsystem.WristClimbCMD(),
      m_elevatorSubsystem.ElevatorClimbDesiredCMD()
    ));
*/
    //Elevate to score in Lvl3
  frc2::JoystickButton(&m_stick, 7).OnTrue(m_elevatorSubsystem.ElevatorLevelThreeCMD());

    //Drop to bottom
  frc2::JoystickButton(&m_stick, 8).OnTrue(
    frc2::cmd::Sequence(
        m_elevatorSubsystem.WristSafeCMD(),
        m_elevatorSubsystem.ElevatorLevelZeroCMD()
      ));

    //Elevate to score in Lvl4
  frc2::JoystickButton(&m_stick, 9).OnTrue(
    frc2::cmd::Sequence(
      m_elevatorSubsystem.WristSafeCMD(),
    m_elevatorSubsystem.ElevatorLevelFourCMD(),
    frc2::cmd::Wait(0.5_s),
    m_elevatorSubsystem.WristDeveliverHighCMD()
  ));
   
    //Elevate to score in Lvl1
  frc2::JoystickButton(&m_stick, 10).OnTrue(m_elevatorSubsystem.ElevatorLevelOneCMD());
    //Elevate to score in Lvl2
  frc2::JoystickButton(&m_stick, 12).OnTrue(m_elevatorSubsystem.ElevatorLevelTwoCMD());

  // Add additional bindings here as needed



}
/*
// Autonomous command placeholder
frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
    return frc2::cmd::Print("No autonomous command configured");
  
}
*/
frc2::Command *RobotContainer::GetAutonomousCommand()
{
    return autoChooser.GetSelected();
}
