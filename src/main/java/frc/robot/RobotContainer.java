/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  // Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_robotDrive.setDefaultCommand(new JoystickDrive(m_robotDrive,() -> -m_driverController.getRawAxis(1), () -> m_driverController.getRawAxis(0)));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
    new JoystickButton(m_driverController, Button.kX.value)
        .whileHeld(new TurnToAngleProfiled(Robot.angle.getDouble(0)+ m_robotDrive.getHeading(), m_robotDrive));

    // Turn to -90 degrees with a profile when the 'A' button is pressed, with a 5 second timeout
    // new JoystickButton(m_driverController, Button.kA.value).whenPressed(new TurnToAngleProfiled(-90, m_robotDrive).withTimeout(1));

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
}
