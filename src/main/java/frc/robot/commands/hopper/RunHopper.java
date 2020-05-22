/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.hopper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HopperSubsystem;

public class RunHopper extends CommandBase {
  /**
   * Creates a new RunHopper.
   */
  private final String mode;
  private final HopperSubsystem m_hopper;
  private double m_speed = 0.6;
  private double h_speed = 0.8;
  private double l_speed = 0.4;
  private double syncspeed = 0.35;

  public RunHopper(final String _mode, final HopperSubsystem _hopper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mode = _mode;
    this.m_hopper = _hopper;
    addRequirements(_hopper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mode == "sync") {
      m_hopper.runHopper("sync", m_speed, 0);
    } else if (mode == "up") {
      m_hopper.runHopper("up", m_speed, 0);
    } else if (mode == "fast_sync") {
      m_hopper.runHopper("sync", h_speed, 0);
    } else if (mode == "down") {
      m_hopper.runHopper("down", 0, l_speed);
    } else if (mode == "slow_sync") {
      m_hopper.runHopper("sync", syncspeed, 0);
    } else {
      m_hopper.runHopper("", -m_speed, m_speed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.runHopper("", 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
