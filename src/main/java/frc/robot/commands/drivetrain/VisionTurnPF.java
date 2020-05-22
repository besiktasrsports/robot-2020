/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class VisionTurnPF extends PIDCommand {
  /**
   * Creates a new VisionTurnPF.
   */
  private DriveSubsystem m_drive;

  public VisionTurnPF(DriveSubsystem drive) {
    super(
        // The controller that the command will use
        new PIDController(DriveConstants.kVisionTurnP, 0, 0),
        // This should return the measurement
        () -> Robot.getVisionYawAngle(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          drive.arcadeDrive(0,
              (Robot.isVisionValid()) ? ((output > 0) ? -DriveConstants.kVisionMinCommand - output : DriveConstants.kVisionMinCommand - output ) : 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_drive = drive;
    addRequirements(m_drive);
    getController().setTolerance(DriveConstants.kTurnToleranceDeg);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Robot.isVisionValid()&&getController().atSetpoint());
  }
}
