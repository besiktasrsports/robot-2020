/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.visionLed;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.VisionLED;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CloseLED extends CommandBase {
  private final VisionLED m_led;

  public CloseLED(VisionLED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_led = led;
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.m_relay.set(false);
  }

  @Override
  public void execute() {
    m_led.m_relay.set(false);
  }
  @Override
  public void end(boolean interrupted) {
    m_led.m_relay.set(true);
  }
}
