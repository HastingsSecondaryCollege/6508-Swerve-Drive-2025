#include "subsystems/CommandSwerveDrivetrain.h"
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/controllers/PPHolonomicDriveController.h>
#include <frc2/command/StartEndCommand.h>
#include "subsystems/CommandSwerveDrivetrain.h"
#include <pathplanner/lib/config/RobotConfig.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>


using namespace subsystems;
using namespace pathplanner;

void CommandSwerveDrivetrain::ConfigureAutoBuilder()
{
    auto config = pathplanner::RobotConfig::fromGUISettings();
    pathplanner::AutoBuilder::configure(
        // Supplier of current robot pose
        [this] { return GetState().Pose; },
        // Consumer for seeding pose against auto
        [this](frc::Pose2d const &pose) { return ResetPose(pose); },
        // Supplier of current robot speeds
        [this] { return GetState().Speeds; },
        // Consumer of ChassisSpeeds and feedforwards to drive the robot
        [this](frc::ChassisSpeeds const &speeds, pathplanner::DriveFeedforwards const &feedforwards) {
            return SetControl(
                m_pathApplyRobotSpeeds.WithSpeeds(speeds)
                    .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
                    .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY)
            );
        },
        std::make_shared<pathplanner::PPHolonomicDriveController>(
            // PID constants for translation
            pathplanner::PIDConstants{10.0, 0.0, 0.0},
            // PID constants for rotation
            pathplanner::PIDConstants{7.0, 0.0, 0.0}
        ),
        std::move(config),
        // Assume the path needs to be flipped for Red vs Blue, this is normally the case
        [] {
            auto const alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
            return alliance == frc::DriverStation::Alliance::kRed;
        },
        this // Subsystem for requirements
    );
}

void CommandSwerveDrivetrain::Periodic()
{
    /*
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is disabled.
     * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
     */
    if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
        auto const allianceColor = frc::DriverStation::GetAlliance();
        if (allianceColor) {
            SetOperatorPerspectiveForward(
                *allianceColor == frc::DriverStation::Alliance::kRed
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        }
    }
}

void CommandSwerveDrivetrain::StartSimThread()
{
    m_lastSimTime = utils::GetCurrentTime();
    m_simNotifier = std::make_unique<frc::Notifier>([this] {
        units::second_t const currentTime = utils::GetCurrentTime();
        auto const deltaTime = currentTime - m_lastSimTime;
        m_lastSimTime = currentTime;

        /* use the measured time delta, get battery voltage from WPILib */
        UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
    });
    m_simNotifier->StartPeriodic(kSimLoopPeriod);
}
/*
frc2::CommandPtr CommandSwerveDrivetrain::MoveForwardsSlowlyCommand(){
    return frc2::cmd::StartEnd(
[this] {Drive(0.2_mps, 0_mps, 0_rad_per_s, false);},
[this] {Drive(0.0_mps, 0_mps, 0_rad_per_s, false); }, {this});
};
//*/
    /*
frc2::CommandPtr CommandSwerveDrivetrain::MoveForwardsSlowlyCommand(){
    return frc2::cmd::StartEnd(
[this] {swerve::requests::FieldCentric().WithVelocityX(0.2_mps).WithVelocityY(0.0_mps).WithRotationalRate(0_rad_per_s);},
[this] {swerve::requests::FieldCentric().WithVelocityX(0_mps).WithVelocityY(0_mps).WithRotationalRate(0_rad_per_s);}, {this});

};
*/
frc2::CommandPtr CommandSwerveDrivetrain::MoveForwardsSlowlyCommand() {
    return frc2::cmd::StartEnd(
        [this] {
            this->SetControl(
                swerve::requests::FieldCentric()
                    .WithVelocityX(0.2_mps) // Forward in X, not Y
                    .WithVelocityY(0.0_mps)
                    .WithRotationalRate(0_rad_per_s));
        },
        [this] {
            this->SetControl(
                swerve::requests::FieldCentric()
                    .WithVelocityX(0_mps)
                    .WithVelocityY(0_mps)
                    .WithRotationalRate(0_rad_per_s));
        },
        {this});
}
frc2::CommandPtr CommandSwerveDrivetrain::MoveBackwardsSlowlyCommand() {
    return frc2::cmd::StartEnd(
        [this] {
            this->SetControl(
                swerve::requests::FieldCentric()
                    .WithVelocityX(-0.2_mps) // Forward in X, not Y
                    .WithVelocityY(0.0_mps)
                    .WithRotationalRate(0_rad_per_s));
        },
        [this] {
            this->SetControl(
                swerve::requests::FieldCentric()
                    .WithVelocityX(0_mps)
                    .WithVelocityY(0_mps)
                    .WithRotationalRate(0_rad_per_s));
        },
        {this});
}
frc2::CommandPtr CommandSwerveDrivetrain::MoveLeftSlowlyCommand() {
    return frc2::cmd::StartEnd(
        [this] {
            this->SetControl(
                swerve::requests::FieldCentric()
                    .WithVelocityX(0.0_mps) // Forward in X, not Y
                    .WithVelocityY(0.2_mps)
                    .WithRotationalRate(0_rad_per_s));
        },
        [this] {
            this->SetControl(
                swerve::requests::FieldCentric()
                    .WithVelocityX(0_mps)
                    .WithVelocityY(0_mps)
                    .WithRotationalRate(0_rad_per_s));
        },
        {this});
}

frc2::CommandPtr CommandSwerveDrivetrain::MoveRightSlowlyCommand() {
    return frc2::cmd::StartEnd(
        [this] {
            this->SetControl(
                swerve::requests::FieldCentric()
                    .WithVelocityX(0.0_mps) // Forward in X, not Y
                    .WithVelocityY(-0.2_mps)
                    .WithRotationalRate(0_rad_per_s));
        },
        [this] {
            this->SetControl(
                swerve::requests::FieldCentric()
                    .WithVelocityX(0_mps)
                    .WithVelocityY(0_mps)
                    .WithRotationalRate(0_rad_per_s));
        },
        {this});
}

//*/