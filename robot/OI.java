package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class OI {

	// Controllers and button boxes
	Joystick xbox1 = new Joystick(RobotMap.p_xbox1); // this is the xbox controller object
	Joystick driverBoard1 = new Joystick(RobotMap.p_driverBoard1); // this is one section of the driver board
	Joystick driverBoard2 = new Joystick(RobotMap.p_driverBoard2);// this is the other section of the driver board

	// xbox1 buttons
	JoystickButton a = new JoystickButton(xbox1, 1); 
	JoystickButton b = new JoystickButton(xbox1, 2);
	JoystickButton x = new JoystickButton(xbox1, 3);
	JoystickButton y = new JoystickButton(xbox1, 4);
	JoystickButton lb = new JoystickButton(xbox1, 5);
	JoystickButton rb = new JoystickButton(xbox1, 6);
	JoystickButton back = new JoystickButton(xbox1, 7);
	JoystickButton start = new JoystickButton(xbox1, 8);

	// gameboard buttons 
	// public static int p_turn45 = 2; // game board 2
	// public static int p_turn90 = 7; // game board 2
	// public static int p_turn180 = 3; // 2
	// public static int p_cargo1 = 4; // 1
	// public static int p_cargo2 = 5; // 1
	// public static int p_cargo3 = 7; // 1
	// public static int p_panel1 = 3; // 1
	// public static int p_panel2 = 6; //1
	// public static int p_panel3 = 5; //2
	// public static int p_floor = 1; //1
	// public static int p_panelout = 4; //2
	// public static int p_panelin = 1; //2
	// public static int p_cargoout = 2; //1
	// public static int p_cargoin = 0; //1
	// public static int p_punch = 0; //2

	public OI() {

	}

	public double getLeftJoystick() { // this is the x axis on the left joystick for the xbox controller
		return xbox1.getRawAxis(1);
	}
	public double getRightJoystick() {// this is the y axis on the left xbox joystick
		return xbox1.getRawAxis(0);
	}
	public double getActualRightJoystickY() { // this is the right joystick y axis for the xbox controller
		return xbox1.getRawAxis(5);
	}
	public double getActualRightJoystickX() { // x axis for the right joystick on the xbox controller  
		return xbox1.getRawAxis(4); 
	}
	public boolean isAButtonPressed(){ // checks to see if the a button is pressed on the xbox controller
		return xbox1.getRawButton(1);
	}
	public boolean isBButtonPressed(){// checks to see if the b button is pressed on the xbox controller
		return xbox1.getRawButton(2);
	}
	public boolean isXButtonPressed(){// checks to see if the x button is pressed on the xbox controller
		return xbox1.getRawButton(3);
	}
	public boolean isRightBumper(){//checks to see if the right bumper button is pressed on the xbox controller
		return xbox1.getRawButton(6);
	}
	public boolean isLeftBumper(){// checks to see if the left bumper button is pressed on the xbox controller
		return xbox1.getRawButton(5);
	}
	public double getRightTrigger(){//checks to see if the right trigger is pressed on the xbox controller
		return xbox1.getRawAxis(3);
	}
	public double getLeftTrigger(){//checks to see if the left trigger is pressed on the xbox controller
		return xbox1.getRawAxis(2);
	}
	public boolean isTurn45(){//checks to see if the turn45 button is pressed on the driver board
		return driverBoard2.getRawButton(RobotMap.p_turn45);
	}
	public boolean isTurn90(){//checks to see if the turn 90 button is pressed on the xbox controller
		return driverBoard2.getRawButton(RobotMap.p_turn90);
	}
	public boolean isTurn180(){//checks to see if the turn180 button is pressed on the driver board
		return driverBoard2.getRawButton(RobotMap.p_turn180);
	}
	public boolean isCargo1(){//checks to see if the cargo1 button is pressed on the driver board
		return driverBoard1.getRawButton(RobotMap.p_cargo1);
	}
	public boolean isCargo2(){//checks to see if the cargo2 button is pressed on the driver board
		return driverBoard1.getRawButton(RobotMap.p_cargo2);
	}
	public boolean isCargo3(){//checks to see if the cargo3 button is pressed on the driver board
		return driverBoard1.getRawButton(RobotMap.p_cargo3);
	}
	public boolean isPanel1(){//checks to see if the isPanel1 button is pressed on the driver board
		return driverBoard1.getRawButton(RobotMap.p_panel1);
	}
	public boolean isPanel2(){
		return driverBoard2.getRawButton(RobotMap.p_panel2);
	}
	public boolean isPanel3(){
		return driverBoard1.getRawButton(RobotMap.p_panel3);
	}
	public boolean isFloor(){
		return driverBoard1.getRawButton(RobotMap.p_floor);
	}
	public boolean isPanelOut(){
		return driverBoard2.getRawButton(RobotMap.p_panelout);
	}
	public boolean isPanelIn(){
		return driverBoard2.getRawButton(RobotMap.p_panelin);
	}
	public boolean isCargoOut(){
		return driverBoard1.getRawButton(RobotMap.p_cargoout);
	}
	public boolean isCargoIn(){
		return driverBoard1.getRawButton(RobotMap.p_cargoin);
	}
	public boolean isPunch(){
		return driverBoard2.getRawButton(RobotMap.p_punch);
	}
	public void checkButtons(){
		SmartDashboard.putBoolean("Turn45:  ", isTurn45());
		SmartDashboard.putBoolean("Turn90:  ", isTurn90());
		SmartDashboard.putBoolean("Turn180:  ", isTurn180());
		SmartDashboard.putBoolean("Cargo1:  ", isCargo1());
		SmartDashboard.putBoolean("Cargo2:  ", isCargo2());
		SmartDashboard.putBoolean("Cargo3:  ", isCargo3());
		SmartDashboard.putBoolean("Panel1:  ", isPanel1());
		SmartDashboard.putBoolean("Panel2:  ", isPanel2());
		SmartDashboard.putBoolean("Panel3:  ", isPanel3());
		SmartDashboard.putBoolean("Floor:  ", isFloor());
		SmartDashboard.putBoolean("PanelOut:  ", isPanelOut());
		SmartDashboard.putBoolean("PanelIn:  ", isPanelIn());
		SmartDashboard.putBoolean("CargoIn:  ", isCargoIn());
		SmartDashboard.putBoolean("CargOut:  ", isCargoOut());
		SmartDashboard.putBoolean("Punch:  ", isPunch());
	}
	public void checkPanel(){
		SmartDashboard.putBoolean("1, 8:  ", driverBoard1.getRawButton(8));// cargo 3
		SmartDashboard.putBoolean("1, 1:  ", driverBoard1.getRawButton(1));// cargo in
		SmartDashboard.putBoolean("1, 2:  ", driverBoard1.getRawButton(2));// floor
		SmartDashboard.putBoolean("1, 3:  ", driverBoard1.getRawButton(3));// cargo out
		SmartDashboard.putBoolean("1, 4:  ", driverBoard1.getRawButton(4));// panel 1
		SmartDashboard.putBoolean("1, 5:  ", driverBoard1.getRawButton(5));// cargo1
		SmartDashboard.putBoolean("1, 6:  ", driverBoard1.getRawButton(6));// cargo2
		SmartDashboard.putBoolean("1, 7:  ", driverBoard1.getRawButton(7));// panel 3
		SmartDashboard.putBoolean("2, 8:  ", driverBoard2.getRawButton(8));// turn 90
		SmartDashboard.putBoolean("2, 1:  ", driverBoard2.getRawButton(1));// punch
		SmartDashboard.putBoolean("2, 2:  ", driverBoard2.getRawButton(2));// panel in
		SmartDashboard.putBoolean("2, 3:  ", driverBoard2.getRawButton(3));// turn 45
		SmartDashboard.putBoolean("2, 4:  ", driverBoard2.getRawButton(4));// turn 180
		SmartDashboard.putBoolean("2, 5:  ", driverBoard2.getRawButton(5));// panel out
		SmartDashboard.putBoolean("2, 6:  ", driverBoard2.getRawButton(6));// panel 2
		SmartDashboard.putBoolean("2, 7:  ", driverBoard2.getRawButton(7));// nothing
	}

}