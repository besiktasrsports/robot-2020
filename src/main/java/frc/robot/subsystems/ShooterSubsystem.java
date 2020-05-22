/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  public boolean isAtSetpoint = false;
  private WPI_VictorSPX shooterMotor1 = new WPI_VictorSPX(ShooterConstants.kShooterMotor1Port);
  private WPI_VictorSPX shooterMotor2 = new WPI_VictorSPX(ShooterConstants.kShooterMotor2Port);

  public final Encoder shooterEncoder = new Encoder(ShooterConstants.kShooterEncoderA,
      ShooterConstants.kShooterEncoderB, ShooterConstants.kShooterEncoderIsReversed);

  public ShooterSubsystem() {
    shooterEncoder.setDistancePerPulse(1.0 / (ShooterConstants.kShooterEncoderPPR));
    shooterMotor1.setInverted(true);
    shooterMotor2.setInverted(true);
    shooterMotor2.follow(shooterMotor1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("shooter/atSetpoint", isAtSetpoint);
    SmartDashboard.putNumber("shooter/RPM", getRPM());
  }

  public void runShooter(double val) {
    shooterMotor1.set(val);
  }

  public void runShooterVoltage(double voltage) {
    shooterMotor1.setVoltage(voltage);
  }

  public double getRPM() {
    return shooterEncoder.getRate() * 60;
  }

}
