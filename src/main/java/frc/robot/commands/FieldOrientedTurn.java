/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * A command that will turn the robot to the specified angle using a motion profile.
 */
public class FieldOrientedTurn extends ProfiledPIDCommand {
  /**
   * Turns to robot to the specified angle using a motion profile.
   *
   * @param targetAngleDegrees The angle to turn to
   * @param drive              The drive subsystem to use
   */

  DriveSubsystem m_drive;
  // double targetHeading;
  public FieldOrientedTurn(double targetAngleDegrees, DriveSubsystem drive) {
    // double a = 5;
    /*
    ProfiledPIDController ctrl = new ProfiledPIDController(DriveConstants.kTurnP, DriveConstants.kTurnI,
    DriveConstants.kTurnD, new TrapezoidProfile.Constraints(DriveConstants.kMaxTurnRateDegPerS,
    DriveConstants.kMaxTurnAccelerationDegPerSSquared));
    
    ProfiledPIDCommand cmd = new ProfiledPIDCommand(ctrl, drive::getHeading, targetAngleDegrees, 
    0, (output, setpoint) -> drive.arcadeDrive(0, (output > 0) ? 0.07+output/12 : -0.07+output/12), drive);
    */
      
    super(
        new ProfiledPIDController(DriveConstants.kTurnP, DriveConstants.kTurnI,
                                  DriveConstants.kTurnD, new TrapezoidProfile.Constraints(
            DriveConstants.kMaxTurnRateDegPerS,
            DriveConstants.kMaxTurnAccelerationDegPerSSquared)),
        // Close loop on heading
        drive::getHeading,
        //Robot::getVisionYawAngle,
        // Set reference to target
        targetAngleDegrees,
        // targetAngleDegrees,
        // drive.getTarget(), // Robot.angle.getDouble(0)
        // Pipe output to turn robot
        (output, setpoint) -> drive.arcadeDrive(0, (output > 0) ? DriveConstants.kMinCommand+output/12 : -DriveConstants.kMinCommand+output/12), // min command = 0.07
        // Require the drive
        drive);
  // Supplier <TrapezoidProfile.State> target_supplier;
  // TrapezoidProfile.State state = () -> new TrapezoidProfile.State(0, Robot.getVisionYawAngle()+drive.getHeading());
  // m_goal = () -> new State(goalSource.getAsDouble(), 0);  
  // targetAngleDegrees = 

    // Set the controller to be continuous (because it is an angle controller)
    
    // TODO: Uncomment this
    getController().enableContinuousInput(-180, 180);
    
    // Set the controller tolerance - the delta tolerance ensures the robot is stationary at the
    // setpoint before it is considered as having reached the reference
    getController()
        .setTolerance(DriveConstants.kTurnToleranceDeg, DriveConstants.kTurnRateToleranceDegPerS);

   m_drive = drive;

  }

  /*
  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
    // getController()
    // getController().setGoal(Robot.getVisionYawAngle()+m_drive.getHeading());
    // System.out.println("Command set!");
  }
  */
  @Override
  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
    // this.m_goal = () -> new TrapezoidProfile.State(Robot.getVisionYawAngle()+m_drive.getHeading(),0);

    // getController().setGoal(Robot.getVisionYawAngle()+m_drive.getHeading());
  }
  
  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
}
