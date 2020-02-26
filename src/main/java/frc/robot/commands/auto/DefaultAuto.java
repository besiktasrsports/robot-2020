/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.RunHopper;
import frc.robot.commands.SetShooterRPMPF;
import frc.robot.commands.VisionTurnCG;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionLED;
import frc.robot.trajectories.SneakyTrajectory;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class DefaultAuto extends SequentialCommandGroup {
  /**
   * Creates a new DefaultAuto.
   */
  public DefaultAuto(ShooterSubsystem shooter, DriveSubsystem drive, VisionLED led, HopperSubsystem hopper,
      SneakyTrajectory s_trajectory) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(new VisionTurnCG(shooter, drive, led),
        new SetShooterRPMPF(3000, shooter, false).withTimeout(2).raceWith(new RunHopper("sync", hopper)),
        s_trajectory.getRamsete(s_trajectory.defaultAuto).andThen(() -> drive.tankDriveVolts(0, 0)));
  }
}
