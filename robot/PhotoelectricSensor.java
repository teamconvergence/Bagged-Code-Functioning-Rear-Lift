/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.ControlMode;
// import edu.wpi.first.wpilibj.AnalogInput;

/**
 * Add your docs here.
 */
public class PhotoelectricSensor extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // DigitalInput photoEye = new DigitalInput(RobotMap.p_photoeye);

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public boolean getPhotoEye(){
    // return photoEye.get();
    return false;
   }

}
