/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;

public class ShooterPIDSubsystem extends PIDSubsystem {
  /**
   * Creates a new ShooterPIDSubsystem.
   */
  private WPI_VictorSPX shooterMotor1 = new WPI_VictorSPX(ShooterConstants.kShooterMotor1Port);
  private WPI_VictorSPX shooterMotor2 = new WPI_VictorSPX(ShooterConstants.kShooterMotor2Port);
  private double goal;
  // private final SpeedControllerGroup shooterMotorGroup = new
  // SpeedControllerGroup(shooterMotor1, shooterMotor2);
  private final Encoder shooterEncoder = new Encoder(ShooterConstants.kShooterEncoderA,
      ShooterConstants.kShooterEncoderB, ShooterConstants.kShooterEncoderIsReversed);
  private final SimpleMotorFeedforward m_shooterFeedForward = new SimpleMotorFeedforward(ShooterConstants.kSVolts,
   ShooterConstants.kVoltSecondsPerRotation);    
  public ShooterPIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ShooterConstants.kShootP, ShooterConstants.kShootI, ShooterConstants.kShootD));
        getController().setTolerance(ShooterConstants.kShooterToleranceRPM);
        setSetpoint(goal);

      shooterEncoder.setDistancePerPulse(1.0 / (ShooterConstants.kShooterEncoderPPR));
      shooterMotor1.setInverted(true);
      shooterMotor2.setInverted(true);
      shooterMotor2.follow(shooterMotor1);

  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    shooterMotor1.setVoltage(output + m_shooterFeedForward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return 0;
  }
  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }
  
}
