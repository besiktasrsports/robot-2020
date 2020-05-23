/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {
  /**
   * Creates a new HopperSubsystem.
   */
  private String hopperState = "";
  private final WPI_VictorSPX hopperMotor1 = new WPI_VictorSPX(HopperConstants.kHopperMotor1Port);
  private final WPI_VictorSPX hopperMotor2 = new WPI_VictorSPX(HopperConstants.kHopperMotor2Port);

  public HopperSubsystem() {
    hopperMotor1.setInverted(true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putString("hopper/hopperState", hopperState);
  }

  public void runHopper(String _mode, double _speed1, double _speed2) {
    if (_mode == "sync") {
      hopperMotor1.set(_speed1);
      hopperMotor2.set(-_speed1);
    } else if (_mode == "up") {
      hopperMotor1.set(_speed1);
      hopperMotor2.set(0);
    } else if (_mode == "down") {
      hopperMotor1.set(0);
      hopperMotor2.set(_speed2);
    } else {
      hopperMotor1.set(_speed1);
      hopperMotor2.set(_speed2);
    }
    hopperState = _mode;

  }
}
