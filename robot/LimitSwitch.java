/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class LimitSwitch extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // DigitalInput leftLimit = new DigitalInput(RobotMap.p_limitleft);
  // DigitalInput rightLimit = new DigitalInput(RobotMap.p_limitright);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  // public boolean leftLimitPosition(){
  //   return leftLimit.get();
  // }
  // public boolean rightLimitPosition(){
  //   return rightLimit.get();
  // }
}
