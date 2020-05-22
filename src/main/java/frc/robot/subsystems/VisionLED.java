/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MiscConstants;

public class VisionLED extends SubsystemBase {
  /**
   * Creates a new VisionLED.
   */
  private boolean ledState;
  public final DigitalOutput m_relay = new DigitalOutput(MiscConstants.kLEDRelayPort);
  public VisionLED() {
    toggleRelay(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    ledState = !m_relay.get();
    SmartDashboard.putBoolean("led/ledState", ledState);
  }

  
  public void toggleRelay(boolean _status) {
    m_relay.set(_status);
  }
}
