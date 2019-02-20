/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
/**
 * Add your docs here.
 */
public class LimelightCamera extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry ts = table.getEntry("ts");
  NetworkTableEntry tshort = table.getEntry("tshort");
  NetworkTableEntry tlong = table.getEntry("tlong");
  NetworkTableEntry thor = table.getEntry("thor");
  NetworkTableEntry tvert = table.getEntry("tvert");

  double x = tx.getDouble(0.0);
  double y = ty.getDouble(0.0);
  double area = ta.getDouble(0.0);
  double skew = ts.getDouble(0.0);
  double m_tshort = tshort.getDouble(0.0);
  double m_tlong = tlong.getDouble(0.0);
  double m_thor = thor.getDouble(0.0);
  double m_tvert = tvert.getDouble(0.0);
  // boolean visible = tv.getBoolean(true);

  public LimelightCamera(){
    // nothing to do 
  }

  public void readCamera(){
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    // visible = tv.getBoolean(false);
    skew = ts.getDouble(0.0);
    m_thor = thor.getDouble(0.0);
    m_tvert = tvert.getDouble(0.0);
    m_tshort = tshort.getDouble(0.0);
    m_tlong = tlong.getDouble(0.0);
  }

  public void updateDashboard(){  
    SmartDashboard.putNumber("LimelightX:  ", x);
    SmartDashboard.putNumber("LimelightY:  ", y);
    // SmartDashboard.putNumber("LimelightArea:  ", area);
    SmartDashboard.putBoolean("LimelightTargetVisible:  ", isTargetVisible());
    // SmartDashboard.putNumber("LimelightSkew:   ", skew);
    // SmartDashboard.putNumber("LimelightThor:   ", m_thor);
    // SmartDashboard.putNumber("LimelightTvert:   ", m_tvert);
    // SmartDashboard.putNumber("LimelightTshort:   ", m_tshort);
    // SmartDashboard.putNumber("LimelightTlong:   ", m_tlong);
  }
  public double getX(){
    return tx.getDouble(0.0);
  }
  public double getY(){
    return ty.getDouble(0.0);
  }
  public double getArea(){
    return ta.getDouble(0.0);
  }
  public boolean isTargetVisible(){
    if(getArea()>0){
      return true;
    }
    return false;
  }
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
