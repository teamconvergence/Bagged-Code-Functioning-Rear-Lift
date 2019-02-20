/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.I2C.Port;

// import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
// import edu.wpi.first.wpilibj.RobotDrive;
// import edu.wpi.first.wpilibj.SampleRobot;
// import edu.wpi.first.wpilibj.Timer;

/**
 * Add your docs here.
 */
public class Gyroscope extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // public AnalogGyro gyro = new AnalogGyro(0);
  public AHRS gyro = new AHRS(Port.kMXP);
  public PIDController turnController;
  static final double kP = 0.1;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;
    
  static final double kToleranceDegrees = 2.0f;


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void init(PIDOutput driveTrain){
    turnController = new PIDController(kP, kI, kD, kF, gyro, driveTrain);
        turnController.setInputRange(-180.0f,  180.0f);
        turnController.setOutputRange(-0.7, 0.7);
        turnController.setAbsoluteTolerance(kToleranceDegrees);
        turnController.setContinuous(true);
        gyro.reset();
  }
  public void reset(){
    gyro.reset();
  }
  public void updateGyro(){
    SmartDashboard.putNumber("GyroScope Angle:  ", gyro.getAngle());
  }
  public double getGyroAngle(){
    return gyro.getAngle();
  }
  public void enable(){
    turnController.enable();
  }
  public void disable(){
    turnController.disable();
  }
  public void setSetPoint(double setpoint){
    turnController.setSetpoint(setpoint);
  }
  public boolean onTarget(){
    return turnController.onTarget();
  }
}
