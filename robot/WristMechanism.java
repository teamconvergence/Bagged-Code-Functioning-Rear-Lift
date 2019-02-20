/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;

import edu.wpi.first.wpilibj.Encoder;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class WristMechanism extends Subsystem implements PIDOutput{
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  // public Encoder wristEncoder = new Encoder(RobotMap.p_secondencoderchannel1, RobotMap.p_secondencoderchannel2, false, Encoder.EncodingType.k4X);
  public TalonSRX wristMotor1 = new TalonSRX(RobotMap.p_wristmotor1);
  public TalonSRX wristMotor2 = new TalonSRX(RobotMap.p_wristmotor2);

  public WPI_TalonSRX intakeMotor = new WPI_TalonSRX(RobotMap.p_intakemotor);

  public double speed = 0.5;

  public PIDController wristController;

  static final double kP = 0.1;
  static final double kI = 0.00;
  static final double kD = 0.00;
  static final double kF = 0.00;
  static final double kToleranceDegrees = 2.0f;

  public double turn45Encoder = 500;
  public double turn90Encoder = 0;
  public double turn180Encoder = -1000;

  public WristMechanism(){
    wristMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    wristMotor2.follow(wristMotor1);
    wristMotor2.setInverted(true);
    wristMotor1.setNeutralMode(NeutralMode.Brake);
    wristMotor2.setNeutralMode(NeutralMode.Brake);

  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void init(PIDOutput wrist){
    // wristController = new PIDController(kP, kI, kD, getEncoderCount(), wrist);
    //   wristController.setInputRange(-180.0f,  180.0f);
    //   wristController.setOutputRange(-0.15, 0.15);
    //   wristController.setAbsoluteTolerance(kToleranceDegrees);
    //   wristController.setContinuous(true);
  }
  public void resetEncoder(){
    wristMotor1.setSelectedSensorPosition(0);
  }
  public void moveWristMotor(double speed){
    wristMotor1.set(ControlMode.Position, speed);
  }
  public void intakeIn(){
    intakeMotor.set(speed);
  }
  public void intakeOut(){
    intakeMotor.set(-speed);
  }
  public void stopIntake(){
    intakeMotor.set(0.0);
  }

  public double getEncoderCount(){
    return wristMotor1.getSelectedSensorPosition();
  }

  public boolean moveWrist45(){
    if(getEncoderCount()>turn45Encoder+100){
      moveWristMotor(-0.15);
      return false;
    }
    else if(getEncoderCount()<turn45Encoder-100){
      moveWristMotor(0.15);
      return false;
    }
    else {
      moveWristMotor(0.0);
      return true;
    }
  }
  public boolean moveWrist90(){
    if(getEncoderCount()>turn90Encoder+100){
      moveWristMotor(-0.15);
      return false;
    }
    else if(getEncoderCount()<turn90Encoder-100){
      moveWristMotor(0.15);
      return false;
    }
    else {
      moveWristMotor(0.0);
      return true;
    }
  }
  public boolean moveWrist180(){
    if(getEncoderCount()>turn180Encoder+100){
      moveWristMotor(-0.15);
      return false;
    }
    else if(getEncoderCount()<turn180Encoder-100){
      moveWristMotor(0.15);
      return false;
    }
    else {
      moveWristMotor(0.0);
      return true;
    }
    }
    @Override
  public void pidWrite(double output) {
    speed = output;
    SmartDashboard.putNumber("WristMotor PidWrite:  ", output);
  }
  public void turnAngle(){
     moveWristMotor(speed);
  }
}
