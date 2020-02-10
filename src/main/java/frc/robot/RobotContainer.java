/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
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
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  public final WoFSubsystem m_wof = new WoFSubsystem();
  
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
    new JoystickButton(m_driverController, 3).whileHeld(new VisionTurnProfiled(m_robotDrive));
    // Field oriented drive
    new POVButton(m_driverController, 0).whileHeld(new FieldOrientedTurn(0, m_robotDrive));
    new POVButton(m_driverController, 45).whileHeld(new FieldOrientedTurn(45, m_robotDrive));
    new POVButton(m_driverController, 90).whileHeld(new FieldOrientedTurn(90, m_robotDrive));
    new POVButton(m_driverController, 135).whileHeld(new FieldOrientedTurn(135, m_robotDrive));
    new POVButton(m_driverController, 180).whileHeld(new FieldOrientedTurn(180, m_robotDrive));
    new POVButton(m_driverController, 215).whileHeld(new FieldOrientedTurn(-135, m_robotDrive));
    new POVButton(m_driverController, 270).whileHeld(new FieldOrientedTurn(-90, m_robotDrive));
    new POVButton(m_driverController, 315).whileHeld(new FieldOrientedTurn(-45, m_robotDrive));
    // Shooter commands
    // new JoystickButton(m_driverController, 1).whileHeld(new SetShooterToRPM(50, m_shooter));
    // new JoystickButton(m_driverController, 2).whileHeld(new SetShooterToRPM(30, m_shooter));
    new JoystickButton(m_driverController, 4).whileHeld(new RunShooter(m_shooter, 0.8));
    // Hopper Commands
    new JoystickButton(m_driverController, 1).whileHeld(new RunHopper("sync", m_hopper));
    // Climb commands
    new JoystickButton(m_driverController, 7).whileHeld(new ToggleClimb(m_climb));
    // WoF Commands
    new JoystickButton(m_driverController, 2).whileHeld(new RunWoF(1, m_wof));
    // Intake Commands
    new JoystickButton(m_driverController, 3).whileHeld(new RunIntake(-0.5, m_intake));
    // Misc commands
    // new JoystickButton(m_driverController, 5).whenPressed(new ToggleLED(m_shooter));
    new JoystickButton(m_driverController, 5).whenPressed(new OpenClimb(m_climb));
    new JoystickButton(m_driverController, 6).whenPressed(new CloseClimb(m_climb));
    new JoystickButton(m_driverController, 8).whileHeld(new ToggleCompressor(m_climb));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_chooser.getSelected();
  }

public class m_climb {
}
}
