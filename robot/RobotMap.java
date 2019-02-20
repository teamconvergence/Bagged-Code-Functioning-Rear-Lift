package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	public static int p_xbox1 = 0;
	public static int p_driverBoard1 = 1;
	public static int p_driverBoard2 = 2;

	// DriveTrain Motors
	public static int p_leftDrive1 = 20;// new robot is 20
	public static int p_leftDrive2 = 21;// new robot is 21
	public static int p_rightDrive1 = 22;// new robot is 22
	public static int p_rightDrive2 = 23;// this is supposed to be 13 // new robot is 23
	public static int p_solenoidInPanel = 0;
	public static int p_solenoidOutPanel = 7;
	public static int p_solenoidInPunch = 1;
	public static int p_solenoidOutPunch = 6;
	public static int p_solenoidInLock = 2;
	public static int p_solenoidOutLock = 5;
	public static int p_solenoidInSpare = 3;
	public static int p_solenoidOutSpare = 4;
	public static int p_compressor = 0; // PCM module only works when assigned 0 (previously tried 5)
	public static int p_frontUpperLimit = 0;
	public static int p_frontLowerLimit = 1;
	// public static int p_photoeye = 2;
	public static int p_gyroscope = 0;
	public static int p_elevatormotor1 = 16;
	public static int p_elevatormotor2 = 17;
	public static int p_wristmotor1 = 14;
	public static int p_wristmotor2 = 19;
	public static int p_rearUpperLimit = 3;
	public static int p_rearLowerLimit = 2;
	public static int p_LEDController = 0;
	public static int p_intakemotor = 15;

	//rear motor CAN IDs are starting at 20
	public static int p_rearLiftMotor1 = 18;
	public static int p_rearLiftMotor2 = 12;
	public static int p_rearDriveMotor = 11;




	public static int p_turn45 = 3; // game board 2
	public static int p_turn90 = 8; // game board 2 ?
	public static int p_turn180 = 4; // 2
	public static int p_cargo1 = 5; // 1
	public static int p_cargo2 = 6; // 1
	public static int p_cargo3 = 8; // 1
	public static int p_panel1 = 4; // 1
	public static int p_panel2 = 6; //2
	public static int p_panel3 = 7; //1
	public static int p_floor = 2; //1
	public static int p_panelout = 5; //2
	public static int p_panelin = 2; //2
	public static int p_cargoout = 3; //1
	public static int p_cargoin = 1; //1
	public static int p_punch = 1; //2

}

