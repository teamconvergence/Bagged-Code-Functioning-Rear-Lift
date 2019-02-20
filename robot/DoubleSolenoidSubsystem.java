/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.ControlMode;


/**
 * Add your docs here.
 */
public class DoubleSolenoidSubsystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public Compressor compressor = new Compressor(RobotMap.p_compressor);
  public DoubleSolenoid panel = new DoubleSolenoid(RobotMap.p_solenoidInPanel,RobotMap.p_solenoidOutPanel);
  public DoubleSolenoid punch = new DoubleSolenoid(RobotMap.p_solenoidInPunch,RobotMap.p_solenoidOutPunch);
  public DoubleSolenoid lock = new DoubleSolenoid(RobotMap.p_solenoidInLock,RobotMap.p_solenoidOutLock);
  public DoubleSolenoid spare = new DoubleSolenoid(RobotMap.p_solenoidInSpare,RobotMap.p_solenoidOutSpare);
  

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // reverse();
    // off();
    // compressor.enabled();
    compressor.start();
  }
  public void reversePanel(){
    panel.set(DoubleSolenoid.Value.kReverse);
  }
  public void forwardPanel(){
    panel.set(DoubleSolenoid.Value.kForward);
  }
  public void solenoidOffPanel(){
    panel.set(DoubleSolenoid.Value.kOff);
  }
  
  public void reversePunch(){
    punch.set(DoubleSolenoid.Value.kReverse);
  }
  public void forwardPunch(){
    punch.set(DoubleSolenoid.Value.kForward);
  }
  public void solenoidOffPunch(){
    punch.set(DoubleSolenoid.Value.kOff);
  }

  public void reverseLock(){
    lock.set(DoubleSolenoid.Value.kReverse);
  }
  public void forwardLock(){
    lock.set(DoubleSolenoid.Value.kForward);
  }
  public void solenoidOffLock(){
    lock.set(DoubleSolenoid.Value.kOff);
  }

  public void reverseSpare(){
    spare.set(DoubleSolenoid.Value.kReverse);
  }
  public void forwardSpare(){
    spare.set(DoubleSolenoid.Value.kForward);
  }
  public void solenoidOffSpare(){
    spare.set(DoubleSolenoid.Value.kOff);
  }

  public void compressorOff() {
    compressor.stop();
  }
}
