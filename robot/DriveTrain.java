/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem implements PIDOutput{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  // Motors for DriveTrain
  public CANSparkMax leftDrive1 = new CANSparkMax(RobotMap.p_leftDrive1, MotorType.kBrushless);
  public CANSparkMax leftDrive2 = new CANSparkMax(RobotMap.p_leftDrive2, MotorType.kBrushless);
  // public WPI_TalonSRX leftDrive1 = new WPI_TalonSRX(RobotMap.p_leftDrive1);
  // public WPI_TalonSRX leftDrive2 = new WPI_TalonSRX(RobotMap.p_leftDrive2);
  public CANSparkMax rightDrive1 = new CANSparkMax(RobotMap.p_rightDrive1, MotorType.kBrushless);
  public CANSparkMax rightDrive2 = new CANSparkMax(RobotMap.p_rightDrive2, MotorType.kBrushless);
  // public WPI_TalonSRX rightDrive1 = new WPI_TalonSRX(RobotMap.p_rightDrive1);
  // public WPI_TalonSRX rightDrive2 = new WPI_TalonSRX(RobotMap.p_rightDrive2);
  

  // Left and Right sides for DriveTrain
  // public SpeedController leftController1 = new SpeedController(leftDrive1);
  public SpeedControllerGroup left = new SpeedControllerGroup(leftDrive1, leftDrive2);
  public SpeedControllerGroup right = new SpeedControllerGroup(rightDrive1, rightDrive2);

  // The DriveTrain drive
  // public DifferentialDrive drive1 = new DifferentialDrive(left, right);
  public DifferentialDrive drive1 = new DifferentialDrive(right, left); // this creates the drivetrain object that we call to drive the train, in this case we switched left and right to make it drive straight

  public double turnRate = 0;

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void arcadeDrive(double i, double j) {
    drive1.arcadeDrive(i, j);
  }
  public void tankDrive(double left, double right){
    drive1.tankDrive(left,right);
  }
  @Override
  public void pidWrite(double output) {
    turnRate = output;
    SmartDashboard.putNumber("Gyro PidWrite", output);
  }
  public void turnAngle(){
    drive1.arcadeDrive(0, turnRate);
  }

}