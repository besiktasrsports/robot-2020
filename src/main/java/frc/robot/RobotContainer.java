/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.DriveConstants;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final HopperSubsystem m_hopper = new HopperSubsystem();
  public final ClimbSubsystem m_climb = new ClimbSubsystem();
  public final CellIntakeSubsystem m_intake = new CellIntakeSubsystem();
  // public final ShooterPIDSubsystem m_pidShooter = new ShooterPIDSubsystem();

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_robotDrive.setDefaultCommand(new JoystickDrive(m_robotDrive, () -> -m_driverController.getRawAxis(1),
        () -> m_driverController.getRawAxis(0)));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Vision Drive

    //new JoystickButton(m_driverController, 7).whileHeld(new VisionTurnProfiled(m_robotDrive));

    // Field Oriented Drive

    new POVButton(m_driverController, 0).whileHeld(new FieldOrientedTurn(0, m_robotDrive));
    new POVButton(m_driverController, 45).whileHeld(new FieldOrientedTurn(45, m_robotDrive));
    new POVButton(m_driverController, 90).whileHeld(new FieldOrientedTurn(90, m_robotDrive));
    new POVButton(m_driverController, 135).whileHeld(new FieldOrientedTurn(135, m_robotDrive));
    new POVButton(m_driverController, 180).whileHeld(new FieldOrientedTurn(180, m_robotDrive));
    new POVButton(m_driverController, 215).whileHeld(new FieldOrientedTurn(-135, m_robotDrive));
    new POVButton(m_driverController, 270).whileHeld(new FieldOrientedTurn(-90, m_robotDrive));
    new POVButton(m_driverController, 315).whileHeld(new FieldOrientedTurn(-45, m_robotDrive));

    // Shooter Commands

    // new JoystickButton (m_driverController,3).whileHeld(()-> m_pidShooter.setSetpoint(5),m_pidShooter);
    new JoystickButton (m_driverController,3).whileHeld(new SetShooterRPMPF(3000, m_shooter));
    //new JoystickButton (m_driverController,4).whileHeld(new ShooterSetRPMPID(1500, m_shooter));
    new JoystickButton (m_driverController,4).whileHeld(new RunIntake(0.8, m_intake));
    // Hopper Commands

    new JoystickButton(m_driverController, 2).whileHeld(new RunHopper("", m_hopper));
    new JoystickButton(m_driverController, 1).whileHeld(new RunHopper("sync", m_hopper));

    // Climb Commands

    new JoystickButton(m_driverController, 5).whenPressed(new OpenClimb(m_climb));
    new JoystickButton(m_driverController, 6).whenPressed(new CloseClimb(m_climb));
    new JoystickButton(m_driverController, 8).whileHeld(new ToggleCompressor(m_climb));

    // Intake Commands

    // new JoystickButton(m_driverController, 4).whileHeld(new RunIntake(0.5, m_intake));
    new JoystickButton(m_driverController, 7).whileHeld(new RunShooter(0.65, m_shooter));

    // Misc commands

    // new JoystickButton(m_driverController, 5).whenPressed(new
    // ToggleLED(m_shooter));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return m_chooser.getSelected();
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, 10);

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(DriveConstants.kMaxSpeedMetersPerSecond,
        DriveConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 0.5) // new Translation
        ),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(2, 0, new Rotation2d(0)),
        // Pass config
        config);

    RamseteCommand ramseteCommand = new RamseteCommand(exampleTrajectory, m_robotDrive::getPose,
        new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
            DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics, m_robotDrive::getWheelSpeeds,
        new PIDController(DriveConstants.kPDriveVel, 0, 0), new PIDController(DriveConstants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_robotDrive::tankDriveVolts, m_robotDrive);

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));
  }

}
