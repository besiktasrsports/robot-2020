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
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetShooterToRPM extends ProfiledPIDCommand {
  private static double lastOutput = 0;

  /**
   * Creates a new SetShooterToRPM.
   */
  public SetShooterToRPM(double targetRPM, ShooterSubsystem shooter) {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            ShooterConstants.KShootP, ShooterConstants.KShootI, ShooterConstants.KShootD,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(85, 200)),
        // This should return the measurement
        shooter::getRPM,
        // This should return the goal (can also be a constant)
        targetRPM,
        // This uses the output
        (output, setpoint) -> {shooter.runShooter(lastOutput + output);
        lastOutput = lastOutput+output;},
        shooter
    );

    //getController().enableContinuousInput(0, 85);
    //getController().setTolerance(positionTolerance, velocityTolerance);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
