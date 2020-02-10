/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /**
   * Creates a new ShooterSubsystem.
   */
  private WPI_VictorSPX shooterMotor1 = new WPI_VictorSPX(ShooterConstants.kShooterMotor1Port);
  private WPI_VictorSPX shooterMotor2 = new WPI_VictorSPX(ShooterConstants.kShooterMotor2Port);
  
  private final SpeedControllerGroup shooterMotorGroup = new SpeedControllerGroup(shooterMotor1, shooterMotor2);
  public final Encoder shooterEncoder = new Encoder(ShooterConstants.kShooterEncoderA, ShooterConstants.kShooterEncoderB,
      ShooterConstants.kShooterEncoderIsReversed);
  public final DigitalOutput m_relay= new DigitalOutput(MiscConstants.kLEDRelayPort);


  public ShooterSubsystem() {
    shooterEncoder.setDistancePerPulse(1.0 / (ShooterConstants.kShooterEncoderPPR / 4));
    shooterMotor1.setInverted(true);
    shooterMotor2.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runShooter(double val) {
    shooterMotorGroup.set(val);
    // shooterMotorGroup.setVoltage(val);
  }

  public double getRPM() {
    return shooterEncoder.getRate() * 60;
  }

  public void toggleRelay(boolean _status) {
    m_relay.set(_status);
  }
}
