/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;

public class ClimbSubsystem extends SubsystemBase {
  /**
   * Creates a new ClimbSubsystem.
   */
  private final Compressor compressor = new Compressor(ClimbConstants.kCompressorPort);
  private final DoubleSolenoid climbSolenoid = new DoubleSolenoid(ClimbConstants.kPCMPort,
      ClimbConstants.kClimbDoubleSolenoidPort1, ClimbConstants.kClimbDoubleSolenoidPort2);

  public ClimbSubsystem() {
    compressor.setClosedLoopControl(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(Robot.compressorState);
  }

  /*
   * public void toggleCyclinders(){ boolean status = false; if(status == false){
   * climbSolenoid.set(Value.kForward); status = !status; } else if(status ==
   * true){ climbSolenoid.set(Value.kReverse); status = !status; } }
   */
  public void climberUp() {
    climbSolenoid.set(Value.kForward);
  }

  public void climberDown() {
    climbSolenoid.set(Value.kReverse);
  }

  public void stopCyclinders() {
    climbSolenoid.set(Value.kOff);
  }

  public void openCompressor() {
    compressor.setClosedLoopControl(true);
  }

  public void closeCompressor() {
    compressor.setClosedLoopControl(false);
  }
}
