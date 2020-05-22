/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.hopper.RunHopper;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.shooter.SetShooterRPMPF;
import frc.robot.commands.drivetrain.VisionTurnCG;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionLED;
import frc.robot.SneakyTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CenterRight6Cell extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous.
   */
  public CenterRight6Cell(SneakyTrajectory s_trajectory, ShooterSubsystem shooter, IntakeSubsystem intake,
      HopperSubsystem hopper, DriveSubsystem drive, VisionLED led) {
    super(new SetShooterRPMPF(2900, shooter, true),
        new SetShooterRPMPF(2900, shooter, false).withTimeout(1.5).raceWith(new RunHopper("sync", hopper)),
        s_trajectory.getRamsete(s_trajectory.centerRightAuto[0])
            .raceWith(new RunIntake(0.7, intake)
                .alongWith(new RunHopper("sync", hopper).alongWith(new RunShooter(-0.3, shooter)))),
        s_trajectory.getRamsete(s_trajectory.centerRightAuto[1]).andThen(() -> drive.tankDriveVolts(0, 0)),
        new RunHopper("", hopper).withTimeout(0.2).alongWith(new VisionTurnCG(shooter, drive, led)),
        new SetShooterRPMPF(2900, shooter, false).withTimeout(1.5)
            .raceWith(new RunHopper("sync", hopper).alongWith(new RunIntake(0.7, intake))));
  }
}