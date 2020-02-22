/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static SendableChooser<Integer> autoChooser = new SendableChooser<>();
  private Command m_autonomousCommand;
  // private static Autonomous autoCG;
  private RobotContainer m_robotContainer;
  public static NetworkTableEntry angle;
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("chameleon-vision").getSubTable("picam");
  public static boolean compressorState = false;
  public static boolean climbState = false;
  public static NetworkTableEntry validAngle;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    // m_robotContainer.m_robotDrive.m_gyro.calibrate();
    // m_robotContainer.m_robotDrive.zeroHeading();

    angle = table.getEntry("targetYaw");
    validAngle = table.getEntry("isValid");
    autoChooser.setDefaultOption("Right 6 Ball", 1);
    autoChooser.addOption("Right 8 Ball", 2);
    // autoChooser.addDefault("Auto1", 1);
    // autoChooser.addObject("Auto2", 2);
    SmartDashboard.putData("Autonomous Selector", autoChooser);
    m_robotContainer = new RobotContainer();
    m_robotContainer.m_robotDrive.zeroHeading();

    m_robotContainer.m_shooter.toggleRelay(true);
    // autoCG = new Autonomous();

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    // System.out.println(m_robotContainer.m_robotDrive.getHeading());
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("RPM", m_robotContainer.m_shooter.shooterEncoder.getRate() * 60);
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.m_robotDrive.zeroHeading();
    m_robotContainer.m_robotDrive.resetEncoders();
    m_robotContainer.m_robotDrive.m_odometry
        .resetPosition(m_robotContainer.s_trajectory.centerRightAutoBackwards.getInitialPose(), new Rotation2d(0));
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // int autoMode = autoChooser.getSelected();
    // switch (autoMode) {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    // m_robotContainer.m_robotDrive.zeroHeading();
    // m_robotContainer.m_robotDrive.resetEncoders();
    // m_robotContainer.m_robotDrive.m_odometry.resetPosition(new Pose2d(3.15/2,
    // -2.4/2, new Rotation2d(0)), new Rotation2d(0));
    // m_robotContainer.m_robotDrive.m_odometry.resetPosition(new Pose2d(-1.55/2,
    // -4.1/2, new Rotation2d(0)), new Rotation2d(0));
    m_robotContainer.m_robotDrive.zeroHeading();
    m_robotContainer.m_robotDrive.resetEncoders();
    m_robotContainer.m_robotDrive.m_odometry
        .resetPosition(m_robotContainer.s_trajectory.centerRightAutoBackwards.getInitialPose(), new Rotation2d(0));

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    //System.out.println(m_robotContainer.m_shooter.getRPM());
    //System.out.println(getVisionYawAngle());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public static double getVisionYawAngle() {
    return angle.getDouble(0);
  }
  public static boolean isVisionValid(){
    return validAngle.getBoolean(false);
  }
}
