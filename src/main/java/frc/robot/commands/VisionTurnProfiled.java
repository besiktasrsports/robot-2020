/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A command that will turn the robot to the specified angle using a motion
 * profile.
 */
public class VisionTurnProfiled extends ProfiledPIDCommand {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */

  DriveSubsystem m_drive;

  // double targetHeading;
  public VisionTurnProfiled(DriveSubsystem drive) {

    super(
        new ProfiledPIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD,
            new TrapezoidProfile.Constraints(DriveConstants.kMaxTurnRateDegPerS,
                DriveConstants.kMaxTurnAccelerationDegPerSSquared)),
        // Close loop on heading
        drive::getHeadingCW,
        // Goal is set after the PID controller
        0,
        // Pipe output to turn robot
        (output, setpoint) -> drive.arcadeDrive(0, (Robot.isVisionValid())?((output > 0) ? 0.0 + output / 12 : -0.0 + output / 12):0), // min
                                                                                                           // command =
                                                                                                           // 0.07
        // Require the drive
        drive);
    this.m_goal = () -> new TrapezoidProfile.State(Robot.getVisionYawAngle() + drive.getHeadingCW(), 0);

    getController().setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);

  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}
