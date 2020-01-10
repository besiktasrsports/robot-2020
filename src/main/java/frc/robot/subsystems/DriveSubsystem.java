/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveSubsystem.
   */

  private final WPI_VictorSPX leftRearMotor = new WPI_VictorSPX(DriveConstants.kLeftRearMotorPort);
  private final WPI_VictorSPX leftFrontMotor = new WPI_VictorSPX(DriveConstants.kLeftFrontMotorPort);

  private final WPI_VictorSPX rightRearMotor = new WPI_VictorSPX(DriveConstants.kRightRearMotorPort);
  private final WPI_VictorSPX rightFrontMotor = new WPI_VictorSPX(DriveConstants.kRightFrontMotorPort);

  private final SpeedControllerGroup m_leftMotors =
      new SpeedControllerGroup(leftRearMotor, leftFrontMotor);

  // The motors on the right side of the drive.
  private final SpeedControllerGroup m_rightMotors =
      new SpeedControllerGroup(rightRearMotor, rightFrontMotor);
  // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);
  private final Gyro m_gyro = new ADXRS450_Gyro();
  
  public DriveSubsystem() {
    
    leftRearMotor.setSafetyEnabled(false);
    leftFrontMotor.setSafetyEnabled(false);

    rightRearMotor.setSafetyEnabled(false);
    rightFrontMotor.setSafetyEnabled(false);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.print("Robot angle:");
    // System.out.println(getHeading());
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

}
