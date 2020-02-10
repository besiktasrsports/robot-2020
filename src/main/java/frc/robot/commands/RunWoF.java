/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WoFSubsystem;

public class RunWoF extends CommandBase {
  /**
   * Creates a new RunWoF.
   */
  private final double speed;
  private final WoFSubsystem m_wof;

  public RunWoF(double _speed, WoFSubsystem _wof) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_wof = _wof;
    speed = _speed;
    addRequirements(m_wof);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_wof.runWoF(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_wof.runWoF(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
