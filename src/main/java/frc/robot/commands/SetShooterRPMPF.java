/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

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

  public SetShooterRPMPF(double targetRPM, ShooterSubsystem shooter) {
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
          System.out.print("Output");
          System.out.println(m_motorOutput);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    m_shooter = shooter;
    addRequirements(m_shooter);
  }

  @Override
  public void initialize() {
    // TODO Auto-generated method stub
    super.initialize();
    m_motorOutput = 0;
  }

  @Override
  public void execute() {
    // TODO Auto-generated method stub
    super.execute();
    /*
     * System.out.print("Set point:");
     * System.out.println(this.m_controller.getSetpoint());
     * System.out.print("Position Error: ");
     * System.out.println(this.m_controller.getPositionError());
     */
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    // TODO Auto-generated method stub
    super.end(interrupted);
    m_shooter.runShooterVoltage(0);
  }
}
