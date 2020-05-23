/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
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
  NetworkTableInstance chameleon = NetworkTableInstance.create();
  public static SendableChooser<Integer> autoChooser = new SendableChooser<>();
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  public static NetworkTableEntry angle;
  public static NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = chameleon.getTable("chameleon-vision").getSubTable("Microsoft LifeCam HD-3000");
  NetworkTable falconDB = inst.getTable("Live_Dashboard");
  public static NetworkTableEntry validAngle;
  NetworkTableEntry robot_x, robot_y, robot_h, traj_x, traj_y, traj_h, is_following, is_locked, turret_angle,matchTime,shooterRPM;
  double startTime, currentTime;
  Pose2d robotPose;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    CameraServer server = CameraServer.getInstance();
    server.startAutomaticCapture();
    chameleon.startClient("10.72.85.12");
    angle = table.getEntry("targetYaw");
    validAngle = table.getEntry("isValid");
    autoChooser.setDefaultOption("3 Cell Straight", 0);
    autoChooser.addOption("Center Right 6 Cell", 1);
    autoChooser.addOption("Left 5 Cell", 2);
    SmartDashboard.putData("Autonomous Selector", autoChooser);
    m_robotContainer = new RobotContainer();
    robot_x = falconDB.getEntry("robotX");
    robot_y = falconDB.getEntry("robotY");
    robot_h = falconDB.getEntry("robotHeading");
    traj_x = falconDB.getEntry("pathX");
    traj_y = falconDB.getEntry("pathY");
    traj_h = falconDB.getEntry("pathHeading");
    is_following = falconDB.getEntry("isFollowingPath");
    is_locked = falconDB.getEntry("isTurretLocked");
    turret_angle = falconDB.getEntry("turretAngle");
    matchTime = falconDB.getEntry("matchTime");
    shooterRPM = falconDB.getEntry("shooterRPM");
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

    CommandScheduler.getInstance().run();
    SmartDashboard.putBoolean("Vision Available", isVisionValid());
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
    if (autoChooser.getSelected() == 2) {
      m_robotContainer.m_robotDrive.m_odometry
          .resetPosition(m_robotContainer.s_trajectory.leftAuto5Cell[0].getInitialPose(), new Rotation2d(0));
    } else {
      m_robotContainer.m_robotDrive.m_odometry
          .resetPosition(m_robotContainer.s_trajectory.centerRightAuto[0].getInitialPose(), new Rotation2d(0));
    }
    m_autonomousCommand = m_robotContainer.getAutonomousCommand(autoChooser.getSelected());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    startTime = Timer.getFPGATimestamp();
    is_following.setBoolean(true);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    currentTime = Timer.getFPGATimestamp() - startTime;
    robotPose = m_robotContainer.s_trajectory.wpilib.sample(currentTime).poseMeters;
    robot_x.setDouble(robotPose.getTranslation().getX());
    robot_y.setDouble(robotPose.getTranslation().getY());
    robot_h.setDouble(robotPose.getRotation().getRadians());
    traj_x.setDouble(robotPose.getTranslation().getX());
    traj_y.setDouble(robotPose.getTranslation().getY());
    traj_h.setDouble(robotPose.getRotation().getRadians());
    turret_angle.setDouble(robotPose.getRotation().getRadians()+(currentTime%(Math.PI*2)));
    is_locked.setBoolean(currentTime%2>1 ? true : false);
    shooterRPM.setDouble((currentTime*400)%5000);
    matchTime.setDouble(currentTime);

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
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

  public static boolean isVisionValid() {
    return validAngle.getBoolean(false);
  }
}
