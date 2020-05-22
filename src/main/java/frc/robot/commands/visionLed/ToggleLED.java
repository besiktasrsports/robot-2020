/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.visionLed;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionLED;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ToggleLED extends InstantCommand {
  private final VisionLED m_led;

  public ToggleLED(VisionLED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_led = led;
    addRequirements(m_led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_led.toggleRelay(!m_led.m_relay.get());
  }
}
