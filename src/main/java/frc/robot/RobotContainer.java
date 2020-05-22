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

import frc.robot.commands.climber.*;
import frc.robot.commands.drivetrain.*;
import frc.robot.commands.hopper.*;
import frc.robot.commands.intake.*;
import frc.robot.commands.shooter.*;
import frc.robot.commands.visionLed.*;
import frc.robot.commands.auto.CenterRight6Cell;
import frc.robot.commands.auto.CenterRight8Cell;
import frc.robot.commands.auto.DefaultAuto;
import frc.robot.commands.auto.Left5Cell;
import frc.robot.commands.auto.Left6Cell;
import frc.robot.commands.auto.Right6Cell;
import frc.robot.commands.auto.Right8Cell;
import frc.robot.subsystems.*;
import frc.robot.SneakyTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
  public final VisionLED m_visionLed = new VisionLED();
  public final SneakyTrajectory s_trajectory = new SneakyTrajectory(m_robotDrive);
  // public final ShooterPIDSubsystem m_pidShooter = new ShooterPIDSubsystem();

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  public Joystick m_driverController = new Joystick(OIConstants.kDriverControllerPort);
  public Joystick m_operatorController = new Joystick(OIConstants.kOperatorControllerPort);

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
    
    new JoystickButton(m_driverController, 5).toggleWhenPressed(new SetShooterRPMPF(2800, m_shooter, false));
    new JoystickButton(m_driverController, 6).toggleWhenPressed(new SetShooterRPMPF(3000, m_shooter, false));
    new JoystickButton(m_driverController, 3).whileHeld(new VisionTurnPF(m_robotDrive));
    new JoystickButton(m_driverController, 3).whileHeld(new CloseLED(m_visionLed));

    // Hopper Commands

    new JoystickButton(m_driverController, 2).whileHeld(new RunHopper("", m_hopper));
    new JoystickButton(m_driverController, 1).whileHeld(new RunHopper("sync", m_hopper));
    new JoystickButton(m_driverController, 4).whileHeld(new RunHopper("fast_sync", m_hopper));

    // Climb Commands

    new JoystickButton(m_operatorController, 5).whenPressed(new OpenClimb(m_climb));
    new JoystickButton(m_operatorController, 6).whenPressed(new CloseClimb(m_climb));
    new JoystickButton(m_operatorController, 10).whileHeld(new ToggleCompressor(m_climb));

    // Intake Commands

    new JoystickButton(m_driverController, 1).whileHeld(new RunIntake(0.7, m_intake));
    new JoystickButton(m_operatorController, 1).whileHeld(new RunIntake(-0.5, m_intake));
    new JoystickButton(m_operatorController, 4).whileHeld(new RunShooter(0.65, m_shooter));
    new JoystickButton(m_operatorController, 2).whileHeld(new RunShooter(-0.3, m_shooter));

    // Misc commands

    new JoystickButton(m_driverController, 10).whenPressed(new ToggleLED(m_visionLed));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(Integer auto) {
    // An ExampleCommand will run in autonomous
    switch (auto) {
    case 1:
      return new CenterRight6Cell(s_trajectory, m_shooter, m_intake, m_hopper, m_robotDrive, m_visionLed);
    case 2:
      return new Left5Cell(s_trajectory, m_shooter, m_intake, m_hopper, m_robotDrive, m_visionLed);
    case 3:
      return new Right6Cell(s_trajectory, m_shooter, m_intake, m_hopper, m_robotDrive, m_visionLed);
    case 4:
      return new Right8Cell(s_trajectory, m_shooter, m_intake, m_hopper, m_robotDrive, m_visionLed);
    case 5:
      return new CenterRight8Cell(s_trajectory, m_shooter, m_intake, m_hopper, m_robotDrive, m_visionLed);
    case 6:
      return new Left6Cell(s_trajectory, m_shooter, m_intake, m_hopper, m_robotDrive, m_visionLed);
    default:
      return new DefaultAuto(m_shooter, m_robotDrive, m_visionLed, m_hopper);
    }
  }

}
