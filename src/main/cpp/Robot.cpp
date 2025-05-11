// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/CommandScheduler.h>
#include <cameraserver/CameraServer.h>
#include "subsystems/LimelightHelpers.h"
#include <iostream>

frc::Field2d m_field;
constexpr units::time::second_t print_period{500_ms};

Robot::Robot() {
  /* Configure CANdi */
  configs::CANdiConfiguration toApply{};

  /* User can change the configs if they want, or leave it empty for factory-default */

  candi.GetConfigurator().Apply(toApply);

  /* Speed up signals to an appropriate rate */
  BaseStatusSignal::SetUpdateFrequencyForAll(100_Hz, candi.GetPWM1Position(), candi.GetPWM1Velocity(), candi.GetS2State());

}

void Robot::RobotInit(){

// Do this in either robot or subsystem init
frc::SmartDashboard::PutData("Field", &m_field);

cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();
camera.SetResolution(640, 480);
camera.SetFPS(30);

if (kUseLimelight) {
    auto const driveState = m_container.drivetrain.GetState();
    auto const heading = driveState.Pose.Rotation().Degrees();
    auto const omega = driveState.Speeds.omega;

    LimelightHelpers::SetRobotOrientation("limelight", heading.value(), 0, 0, 0, 0, 0);
    auto llMeasurement = LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
    if (llMeasurement && llMeasurement->tagCount > 0 && units::math::abs(omega) < 2_tps) {
      m_container.drivetrain.AddVisionMeasurement(llMeasurement->pose, llMeasurement->timestampSeconds);
    }
  }

/*
frc::SmartDashboard::PutNumber("Elevator Level Zero", 0.44);
frc::SmartDashboard::PutNumber("Elevator Level One", 8.0);
frc::SmartDashboard::PutNumber("Elevator Level Two", 25.0);
frc::SmartDashboard::PutNumber("Elevator Level Three", 25.0);
frc::SmartDashboard::PutNumber("Elevator Level Four", 25.0);

frc::SmartDashboard::PutNumber("Elevator Climb Bottom", 25.0);
frc::SmartDashboard::PutNumber("Elevator Climb Top", 25.0);

frc::SmartDashboard::PutNumber("Wrist Home", 0.06);
frc::SmartDashboard::PutNumber("Wrist Safe", 2.65);
frc::SmartDashboard::PutNumber("Wrist To Processor", 16.77);

frc::SmartDashboard::PutNumber("Intake Turns Per Second", -6.0);
frc::SmartDashboard::PutNumber("Delivery Turns Per Second", -6.0);

*/
/*
  frc::Preferences::InitDouble("Elevator Level Zero", 0.44);
  frc::Preferences::InitDouble("Elevator Level One", 8.0);
  frc::Preferences::InitDouble("Elevator Level Two", 25.0);
  frc::Preferences::InitDouble("Elevator Level Three", 25.0);
  frc::Preferences::InitDouble("Elevator Level Four", 25.0);

  frc::Preferences::InitDouble("Elevator Climb Bottom", 25.0);
  frc::Preferences::InitDouble("Elevator Climb Top", 25.0);
  
  frc::Preferences::InitDouble("Wrist Home", 0.06);
  frc::Preferences::InitDouble("Wrist Safe", 2.65);
  frc::Preferences::InitDouble("Wrist To Processor", 16.77);

  frc::Preferences::InitDouble("Intake Turns Per Second", -6.0);
  frc::Preferences::InitDouble("Delivery Turns Per Second", -6.0);
  */
}

void Robot::RobotPeriodic() {

  // Do this in either robot periodic or subsystem periodic
//m_field.SetRobotPose(m_odometry.GetPose());
m_field.SetRobotPose(m_container.drivetrain.GetState().Pose);

 
  frc2::CommandScheduler::GetInstance().Run();
// /* Every print_period get the CANdi position/velocity and report it */
//  if (frc::Timer::GetFPGATimestamp() - currentTime >= print_period) {
//    currentTime += print_period;
//
//    /**
//     * GetPosition automatically calls Refresh(), no need to manually refresh.
//     * 
//     * StatusSignalValues also have the "ostream <<" operator implemented, to provide
//     * a useful print of the signal.
//     */
//    auto &pos = candi.GetPWM1Position();
//    std::cout << "Position is " << pos << " with " << pos.GetTimestamp().GetLatency().value() << " seconds of latency" << std::endl;
//
//    /**
//     * Get the S2 State StatusSignalValue without refreshing
//     */
//    auto &S2State = candi.GetS2State(false);
//    /* This time wait for the signal to reduce latency */
//    S2State.WaitForUpdate(print_period); // Wait up to our period
//    /**
//     * This uses the explicit GetValue and GetUnits functions to print, even though it's not
//     * necessary for the ostream print
//     */
//    std::cout << "S2 State is " <<
//                  S2State.GetValue().ToString() << " " <<
//                  S2State.GetUnits() << " with " <<
//                  S2State.GetTimestamp().GetLatency().value() << " seconds of latency" <<
//                  std::endl;
//    /**
//     * Notice when running this example that the second print's latency is always shorter than the first print's latency.
//     * This is because we explicitly wait for the signal using the WaitForUpdate() method instead of using the Refresh()
//     * method, which only gets the last cached value (similar to how Phoenix v5 works).
//     * This can be used to make sure we synchronously update our control loop from the CAN bus, reducing any latency or jitter in
//     * CAN bus measurements.
//     * When the device is on a CANivore, the reported latency is very close to the true latency of the sensor, as the CANivore
//     * timestamps when it receives the frame. This can be further used for latency compensation.
//     */
//    std::cout << std::endl;
 // }
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    m_autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    m_autonomousCommand->Cancel();
  }
}

void Robot::TeleopPeriodic() {

//Testing Preferences smartdashboard
/*
  double elevatorLevelZero = frc::SmartDashboard::GetNumber("Elevator Level Zero", 0.44);
  double elevatorLevelOne = frc::SmartDashboard::GetNumber("Elevator Level One", 8.0);
  double elevatorLevelTwo = frc::SmartDashboard::GetNumber("Elevator Level Two", 25.0);
  double elevatorLevelThree = frc::SmartDashboard::GetNumber("Elevator Level Three", 25.0);
  double elevatorLevelFour = frc::SmartDashboard::GetNumber("Elevator Level Four", 25.0);

  double elevatorClimbReady = frc::SmartDashboard::GetNumber("Elevator Climb Bottom", 25.0);
  double elevatorClimbDesired = frc::SmartDashboard::GetNumber("Elevator Climb Top", 25.0);

  double wristHome = frc::SmartDashboard::GetNumber("Wrist Home", 0.06);
  double wristSafe = frc::SmartDashboard::GetNumber("Wrist Safe", 2.65);
  double wristProcessor = frc::SmartDashboard::GetNumber("Wrist To Processor", 16.77);

  double intakeTurns = frc::SmartDashboard::GetNumber("Intake Turns Per Second", -6.0);
  double deliveryTurns = frc::SmartDashboard::GetNumber("Delivery Turns Per Second", -6.0);
  */
}

void Robot::TeleopExit() {}

  

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

void Robot::CreateDoublePreferenceKey(std::string_view KeyName, double DefaultKeyValue)
  {
    if (!frc::Preferences::ContainsKey(KeyName)) // Check if it doesn't already exit
    {
      frc::Preferences::InitDouble(KeyName, DefaultKeyValue); // Create it and set to value if it doesn't exit
    }
  }

  /**
 * This function creates an Entry in the Preference Table
 * and sets it to a value if it doesn't already exist.
 */
void Robot::CreateStringPreferenceKey(std::string_view KeyName, std::string_view DefaultKeyValue)
{
  if (!frc::Preferences::ContainsKey(KeyName)) // Check if it doesn't already exit
  {
    frc::Preferences::SetString(KeyName, DefaultKeyValue); // Create it and set to value if it doesn't exit
  }
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
