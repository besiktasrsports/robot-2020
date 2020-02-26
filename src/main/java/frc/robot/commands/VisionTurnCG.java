/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionLED;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class VisionTurnCG extends SequentialCommandGroup {
  /**
   * Creates a new VisionTurnCG.
   */
  public VisionTurnCG(ShooterSubsystem m_shooter, DriveSubsystem m_drive, VisionLED m_led) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new VisionTurnProfiled(m_drive).withTimeout(0.4).raceWith(new CloseLED(m_led)).andThen(() -> m_drive.tankDriveVolts(0, 0)),new SetShooterRPMPF(3000, m_shooter, true));
  }
}
