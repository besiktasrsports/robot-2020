/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimbSubsystem;

public class ToggleCompressor extends CommandBase {
  /**
   * Creates a new ToggleCompressor.
   */
  private final ClimbSubsystem m_climb;

  public ToggleCompressor(ClimbSubsystem _climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_climb = _climb;
    addRequirements(m_climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climb.compressorState = !m_climb.compressorState;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_climb.compressorState == true) {
      m_climb.openCompressor();
    } else {
      m_climb.closeCompressor();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
