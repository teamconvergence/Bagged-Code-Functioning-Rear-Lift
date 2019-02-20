/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  public static DriveTrain drivetrain = new DriveTrain();
  public static OI oi = new OI();
  public static LimelightCamera limelight = new LimelightCamera();

  //constants for autonoumous driving
  public static float kpAim = 0.04f;
  public static float kpDistance = -0.07f;
  public static float minAimCommand = 0.05f;

  //Declaration of USB Camera and Autonomous Command
  UsbCamera cam;
  public static DoubleSolenoidSubsystem sol = new DoubleSolenoidSubsystem();
  public static LimitSwitch limit = new LimitSwitch();
  public static PhotoelectricSensor photoEye = new PhotoelectricSensor();
  public static Gyroscope gyro = new Gyroscope();
  final double joystickXDampen = 0.6;
  final double joystickYDampen = 0.7;
  public boolean turn = false;
  public double encoderDestination = 0.0;
  public boolean checkPosition = false;
  public static ElevatorMechanism elevator = new ElevatorMechanism();
  public static LEDSubsystem LEDSystem = new LEDSubsystem();
  public static RearLiftMechanism rearLiftMechanism = new RearLiftMechanism();
  public static WristMechanism wristMechanism = new WristMechanism();
  public static boolean elevatorInitDone = false;
  public static boolean wristTurnComplete45;
  public static boolean wristTurnComplete90;
  public static boolean wristTurnComplete180;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);

    //Initialize and start the USB Camera
    cam = CameraServer.getInstance().startAutomaticCapture();
    //Set the max FPS of the camera at 60
    cam.setFPS(60);
    sol.initDefaultCommand();
    // sol.reverse();
    gyro.init(drivetrain);
    LEDSystem.rainbow();
    wristMechanism.resetEncoder();
    rearLiftMechanism.initDefaultCommand();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    
    // if(oi.isAButtonPressed()&& limelight.isTargetVisible()){
    if(oi.isAButtonPressed()&& limelight.isTargetVisible()){
      double xAdjust = 0.0;
      final double minSpeedX = 0.25;
      final double minSpeedY = 0.28;
      final double slowSpeedY = 0.4;
      final double xZone = 10.0;
      final double yZone = 1;
      final double maxSpeedY = 1;
      double ySpeed = 0.0;
      
      double currentX = limelight.getX();

      if(currentX>=xZone||-currentX>=xZone){
        xAdjust = kpAim*currentX;
      }
      else if(currentX>0&&currentX<xZone){
        xAdjust = minSpeedX;
      }
      else if(currentX<0&&currentX>-xZone){
        xAdjust = -minSpeedX;
      }
      else{
        xAdjust = 0;
      }
      double currentY = limelight.getY();
      // if(currentY<yZone){
      //   ySpeed = kpDistance*currentY;
      //   if(ySpeed > maxSpeedY){
      //     ySpeed = maxSpeedY;
      //   }
      //   else if(ySpeed < -maxSpeedY){
      //     ySpeed = -maxSpeedY;
      //   }
      // }
      // else if(currentY>=yZone&&currentY<0){
      //   ySpeed = minSpeedY;
      // }
      // else if(currentY>0){
      //   ySpeed=-minSpeedY;
      // }
      // else {
      //   ySpeed = 0;
      // }
      
      
      if(currentY < 0){
        ySpeed = kpDistance*currentY;// f is for caleb
      }
      else if(currentY > 0){
        ySpeed = -minSpeedY;
      }
      else{
        ySpeed = 0;
      }

      if(ySpeed > maxSpeedY){
        ySpeed = maxSpeedY;
      }
      else if(ySpeed < -maxSpeedY){
        ySpeed = -maxSpeedY;
      }
      if(ySpeed < slowSpeedY && ySpeed > 0){
        ySpeed = slowSpeedY;
        if(currentY<0 && currentY > -yZone){
          ySpeed = minSpeedY;
        }
      }
      else if(ySpeed > -slowSpeedY && ySpeed < 0){
        ySpeed = -slowSpeedY;
        if(currentY>0 && currentY<yZone){
          ySpeed = -minSpeedY;
        }
      }

      drivetrain.arcadeDrive(ySpeed, xAdjust);
    }
    else if(oi.isBButtonPressed()){
      double xAdjust = 0.0;
      if(limelight.getX()>1.0){
        xAdjust = kpAim*limelight.getX()-minAimCommand;
      }
      else{
        xAdjust = kpAim*limelight.getX()+minAimCommand;
      }
      if(Math.abs(kpDistance*limelight.getY()) < 0.35 && Math.abs(limelight.getY()) >= 1){
        drivetrain.arcadeDrive(0.35, xAdjust);
      }
      else if(Math.abs(limelight.getY()) < 1){
        drivetrain.arcadeDrive(0, xAdjust);
      }
      else{
        drivetrain.arcadeDrive(kpDistance*limelight.getY()*joystickYDampen, xAdjust);
      }
        
    }
      // IDEAL TURN RATE 0.4 
      // IDEAL SPEED 0.4
      
      // ANY POSITIVE Y CAMERA VALUE WE WANT SPEED AT -0.4
      // ANY NEGATIVE Y CAMERA VALUE GREATER THAN -10 WILL BE 0.4
      // ANYTHING LESS THAN -10 ON THE Y AXIS WILL BE A HIGHER SPEED

      //FOR X AXIS ABOVE |20| GET COMPUTED SPEED
      //FOR X AXIS BELOW |20| SET 0.4
      // drivetrain.arcadeDrive(kpDistance*limelight.getY(), xAdjust);
      // drivetrain.arcadeDrive(0.4, 0);

    // CREATE A FLAG VARIABLE TO TRIGGER A LOOP THAT MAKES IT TURN TO THE GYRO ANGLE. IT RESETS WHEN IT REACHES ITS TARGET.
    //JOYSTICKS WILL INTERUPT THE LOOP THROUGH A ! STATEMENT

    else{
      // if (Math.abs(oi.getLeftJoystick()) > .05 || Math.abs(oi.getActualRightJoystickY()) > .05) { 
      //   drivetrain.tankDrive(-oi.getLeftJoystick(),-oi.getActualRightJoystickY());
      // } 
      if (Math.abs(oi.getLeftJoystick()) > .05 || Math.abs(oi.getActualRightJoystickX()) > .05) { 
        drivetrain.arcadeDrive(-oi.getLeftJoystick()*joystickYDampen, oi.getActualRightJoystickX()*joystickXDampen);
        turn = false;
        gyro.reset();
        gyro.disable(); 
        rearLiftMechanism.moveDriveMotor(-oi.getLeftJoystick()*joystickYDampen);
      }
      else if(oi.getLeftTrigger()>0){
        gyro.enable();
        gyro.setSetPoint(-90);
        // drivetrain.turnAngle();
        SmartDashboard.putBoolean("Left Trigger:   ", true);
        turn = true;
      }
      else if(oi.getRightTrigger()>0){
        gyro.enable();
        gyro.setSetPoint(90);
        // drivetrain.turnAngle();
        SmartDashboard.putBoolean("Right Trigger:   ", true);
        turn = true;
      }
      else {
        drivetrain.arcadeDrive(0, 0);
        SmartDashboard.putBoolean("Left Trigger:   ", false);
        SmartDashboard.putBoolean("Right Trigger:   ", false);
        rearLiftMechanism.moveDriveMotor(0.0);
      }

      if(turn){
        drivetrain.turnAngle();
      }
      if(gyro.onTarget()){
        turn = false;
        gyro.reset();
        gyro.disable();
      }
    }
    
    if(oi.isFloor()){ // if the floor button is pressed then the elevator will move to the floor along with reseting the encoder
      if(elevator.getEncoderCount()!=ElevatorMechanism.FLOOR_POSITION){
        // elevator.goFloor();
        // elevator.setSpeed(0.);
        // checkPosition = true;
        encoderDestination = ElevatorMechanism.FLOOR_POSITION;
      }
    }
    else if(oi.isCargo1()){
      if(elevator.getEncoderCount()!=ElevatorMechanism.CARGO1_POSITION){
        // elevator.setEncoderDestination(ElevatorMechanism.CARGO1_POSITION);
        // checkPosition = true;
        encoderDestination = ElevatorMechanism.CARGO1_POSITION;
        // elevator.setSpeed(0.2);
      }
      // else{
      //   elevator.setSpeed(0);
      // }
    }
    else if(oi.isCargo2()){
      if(elevator.getEncoderCount()!=ElevatorMechanism.CARGO2_POSITION){
        // elevator.setEncoderDestination(ElevatorMechanism.CARGO2_POSITION);
        // checkPosition = true;
        encoderDestination = ElevatorMechanism.CARGO2_POSITION;
        // elevator.setSpeed(0.2);
      }
      // else{
      //   elevator.setSpeed(0);
      // }
    }
    else if(oi.isCargo3()){
      if(elevator.getEncoderCount()!=ElevatorMechanism.CARGO3_POSITION){
        // elevator.setEncoderDestination(ElevatorMechanism.CARGO3_POSITION);
        // checkPosition = true;
        encoderDestination = ElevatorMechanism.CARGO3_POSITION;
      //   elevator.setSpeed(0.2);
      // }
      // else{
      //   elevator.setSpeed(0);
      }
    }
    else if(oi.isPanel1()){
      if(elevator.getEncoderCount()!=ElevatorMechanism.PANEL1_POSITION){
        // elevator.setEncoderDestination(ElevatorMechanism.PANEL1_POSITION);
        // checkPosition = true;
        encoderDestination = ElevatorMechanism.PANEL1_POSITION;
      //   elevator.setSpeed(0.2);
      // }
      // else{
      //   elevator.setSpeed(0);
      }
    }
    else if(oi.isPanel2()){
      if(elevator.getEncoderCount()!=ElevatorMechanism.PANEL2_POSITION){
        // elevator.setEncoderDestination(ElevatorMechanism.PANEL2_POSITION);
        // checkPosition = true;
        encoderDestination = ElevatorMechanism.PANEL2_POSITION;
      //   elevator.setSpeed(0.2);
      // }
      // else{
      //   elevator.setSpeed(0);
      }
      
    }
    else if(oi.isPanel3()){
      if(elevator.getEncoderCount()!=ElevatorMechanism.PANEL3_POSITION){
        // elevator.setEncoderDestination(ElevatorMechanism.PANEL3_POSITION);
        // checkPosition = true;
        encoderDestination = ElevatorMechanism.PANEL3_POSITION;
      //   elevator.setSpeed(0.2);
      // }
      // else{
      //   elevator.setSpeed(0);
      }
    }
    if(encoderDestination >= elevator.getEncoderCount() - 200 && encoderDestination <= elevator.getEncoderCount() +200){
      elevator.setSpeed(0);
      if(encoderDestination == ElevatorMechanism.FLOOR_POSITION){
        elevator.resetEncoder();
      }
    }
    else{
      elevator.setEncoderDestination(encoderDestination);
    }


    if(oi.isCargoIn()){
      wristMechanism.intakeIn();
    }
    else if(oi.isCargoOut()){
      wristMechanism.intakeOut();
    }
    else {
      wristMechanism.stopIntake();
    }

    if(oi.isPanelIn()){
      sol.reversePanel();
    }
    else if(oi.isPanelOut()){
      sol.forwardPanel();
    }
    else{
      sol.solenoidOffPanel();
    }

    if(oi.isPunch()){
      sol.forwardPunch();
    }
    else{
      sol.reversePunch();
    }

    if(oi.isTurn45()){
      wristMechanism.moveWrist45();
      wristTurnComplete45 = false;
    }
    else if(oi.isTurn90()){
      wristMechanism.moveWrist90();
      wristTurnComplete90 = false;
    }
    else if(oi.isTurn180()){
      wristMechanism.moveWrist180();
      wristTurnComplete180 = false;
    }

    if(!wristTurnComplete45){
      wristTurnComplete45 = wristMechanism.moveWrist45();
    }
    else if(!wristTurnComplete90){
      wristTurnComplete90 = wristMechanism.moveWrist90();
    }
    else if(!wristTurnComplete180){
      wristTurnComplete180 = wristMechanism.moveWrist180();
    }

    if(!elevatorInitDone && oi.isXButtonPressed()){
      elevatorInitDone = elevator.setZeroPosition();
    }

    rearLiftMechanism.checkLimits();

    if(oi.isRightBumper()){
      rearLiftMechanism.raise();
      // rearLiftMechanism.moveLiftMotor(0.2);
      // LEDSystem.pattern(0.77);
      LEDSystem.pattern(0.29);
    }
    else if(oi.isLeftBumper()){
      rearLiftMechanism.lower();
      // LEDSystem.pattern(0.63);
      LEDSystem.pattern(0.09);
    }
    else {
      rearLiftMechanism.moveLiftMotor(0.0);
    }

    // if(oi.isRightBumper()){
    //   LEDSystem.pattern(0.09);
    // }
    // else if(oi.isLeftBumper()){
    //   LEDSystem.pattern(0.29);
    // }


    // if(photoEye.getPhotoEye()){
    //   sol.reverse();
    // }

    gyro.updateGyro();

    SmartDashboard.putBoolean("Photo Eye:  ", photoEye.getPhotoEye());
    SmartDashboard.putBoolean("turn:   ", turn);
    SmartDashboard.putNumber("Elevator Encoder Count:   ", elevator.getEncoderCount());
    SmartDashboard.putNumber("Wrist Encoder Count:   ", wristMechanism.getEncoderCount());
    SmartDashboard.putBoolean("rearUpperLimit:   ", rearLiftMechanism.rearUpperLimit.get());
    SmartDashboard.putBoolean("rearLowerLimit:   ", rearLiftMechanism.rearLowerLimit.get());
    SmartDashboard.putBoolean("frontUpperLimit:   ", elevator.upperLimit.get());
    SmartDashboard.putBoolean("frontLowerLimit:   ", elevator.lowerLimit.get());
    SmartDashboard.putBoolean("isLowered:   ", rearLiftMechanism.isLowered());
    SmartDashboard.putBoolean("Right Bumper:   ", false);
    SmartDashboard.putBoolean("Left Bumper:   ", false);
    limelight.readCamera();
    limelight.updateDashboard();
    // oi.checkButtons();
    oi.checkPanel();

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
    if(!elevatorInitDone && oi.isXButtonPressed()){
      elevatorInitDone = elevator.setZeroPosition();
    }
    else if(oi.isAButtonPressed() && !elevator.upperLimit.get()){
      elevator.setSpeed(0.15);
    }
    if(elevator.upperLimit.get()){
      elevator.setSpeed(0.0);
    }
    SmartDashboard.putNumber("Elevator Upper Limit:  ", elevator.getEncoderCount());
  }
}