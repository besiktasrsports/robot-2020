/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SetShooterRPMPF extends PIDCommand {
  /**
   * Creates a new ShooterSetRPMPID.
   */
  private ShooterSubsystem m_shooter;
  // private static double lastOutput = 0;
  private final static SimpleMotorFeedforward m_shooterFeedForward = new SimpleMotorFeedforward(ShooterConstants.kS,
      ShooterConstants.kV, ShooterConstants.kA);
  private static double m_motorOutput;
  private boolean isInterruptable;

  public SetShooterRPMPF(double targetRPM, ShooterSubsystem shooter, boolean _isInterruptable) {
    super(
        // The controller that the command will use
        new PIDController(ShooterConstants.kShootP, ShooterConstants.kShootI, ShooterConstants.kShootD),
        // This should return the measurement
        shooter::getRPM,
        // This should return the setpoint (can also be a constant)
        targetRPM,
        // This uses the output
        output -> {
          // Use the output here
          m_motorOutput = output + m_shooterFeedForward.calculate(targetRPM);
          shooter.runShooterVoltage(m_motorOutput);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(100);
    m_shooter = shooter;
    this.isInterruptable = _isInterruptable;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    super.initialize();
    m_motorOutput = 0;
  }

  @Override
  public void execute() {
    super.execute();
    m_shooter.isAtSetpoint = getController().atSetpoint();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isInterruptable && getController().atSetpoint()) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    if (!isInterruptable) {
      m_shooter.runShooterVoltage(0);
      m_shooter.isAtSetpoint = false;
    }
  }
}
