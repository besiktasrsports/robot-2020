/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
  /**
   * Creates a new DriveSubsystem.
   */

  private final WPI_VictorSPX leftRearMotor = new WPI_VictorSPX(DriveConstants.kLeftRearMotorPort);
  private final WPI_TalonSRX leftFrontMotor = new WPI_TalonSRX(DriveConstants.kLeftFrontMotorPort);

  private final WPI_VictorSPX rightRearMotor = new WPI_VictorSPX(DriveConstants.kRightRearMotorPort);
  private final WPI_TalonSRX rightFrontMotor = new WPI_TalonSRX(DriveConstants.kRightFrontMotorPort);

  static private int PIDIDX = 0;

  private final DifferentialDrive m_drive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
  private final DifferentialDriveOdometry m_odometry;
  public final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  // private final BuiltInAccelerometer m_accel = new BuiltInAccelerometer();
  private double angular_velocity;
  private double target;

  public DriveSubsystem() {

    leftRearMotor.follow(leftFrontMotor);
    rightRearMotor.follow(rightFrontMotor);

    leftRearMotor.setSafetyEnabled(false);
    leftFrontMotor.setSafetyEnabled(false);
    rightRearMotor.setSafetyEnabled(false);
    rightFrontMotor.setSafetyEnabled(false);
    m_drive.setSafetyEnabled(false);

    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDIDX, 10);
    rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, PIDIDX, 10);
    // leftFrontMotor.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // rightFrontMotor.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    resetEncoders();
    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.print("Robot angle:");
    angular_velocity = m_gyro.getRate();
    SmartDashboard.putNumber("Angular velocity", angular_velocity);
    m_odometry.update(Rotation2d.fromDegrees(getHeading()), leftFrontMotor.getSelectedSensorPosition(),
        rightFrontMotor.getSelectedSensorPosition());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftFrontMotor.getSelectedSensorVelocity(),
        rightFrontMotor.getSelectedSensorVelocity());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftFrontMotor.setVoltage(leftVolts);
    rightFrontMotor.setVoltage(-rightVolts); // eksi
    m_drive.feed();
  }

  public double getAverageEncoderDistance() {
    return (leftFrontMotor.getSelectedSensorPosition() + rightFrontMotor.getSelectedSensorPosition()) / 2.0;
  }

  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  public double getTarget() {
    return getHeading() + target;
  }

  public void setTarget(double val) {
    target = val;
  }

  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot, true);
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

  public void resetEncoders() {
    rightFrontMotor.setSelectedSensorPosition(0);
    leftFrontMotor.setSelectedSensorPosition(0);
  }

}
