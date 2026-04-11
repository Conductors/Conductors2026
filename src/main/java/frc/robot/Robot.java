// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.channels.Pipe;
import java.util.Vector;

import org.opencv.core.Mat;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.commands.climberCommand;
import frc.robot.commands.driveSidewaysPID;
import frc.robot.commands.driveSpinwaysPID;
import frc.robot.commands.driveStraightPID;
import frc.robot.commands.driveToPositionPID;
import frc.robot.commands.extendIntake;
import frc.robot.commands.intakeFuelCmd;
import frc.robot.commands.retractAndIntake;
import frc.robot.commands.retractIntake;
import frc.robot.commands.setClimbSpeed;
import frc.robot.commands.setShooterSpeed;
import frc.robot.commands.turnTowardsAprilPID;
import frc.robot.commands.climberCommand.climbLevel;
import frc.robot.commands.driveBackTillPositioned;
import frc.robot.subsystems.intake;
import frc.robot.subsystems.shooterSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.climbSubsystem;

public class Robot extends TimedRobot {
  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final GenericHID m_buttonBoard = new GenericHID(1);
  private Trigger yButton     = m_controller.y(); 
  private Trigger xButton     = m_controller.x(); 
  private Trigger aButton     = m_controller.a(); 
  private Trigger bButton     = m_controller.b(); 
  private Trigger startButton = m_controller.start(); 
  private Trigger backButton  = m_controller.back();
  private Trigger lbButton    = m_controller.leftBumper();
  private Trigger rbButton    = m_controller.rightBumper(); 
  private Trigger lBTrigger   = m_controller.leftTrigger(0.1); 
  private Trigger rBTrigger   = m_controller.rightTrigger(.1); 
  private Trigger povUp       = m_controller.povUp();
  private Trigger povDown     = m_controller.povDown();
  private Trigger povLeft     = m_controller.povLeft();
  private Trigger povRight    = m_controller.povRight();
  
  private Trigger whiteOne    = new JoystickButton(m_buttonBoard, 1);
  private Trigger redOne      = new JoystickButton(m_buttonBoard, 2);
  private Trigger yellowOne   = new JoystickButton(m_buttonBoard, 3);
  private Trigger greenOne    = new JoystickButton(m_buttonBoard, 4);  
  private Trigger blueOne     = new JoystickButton(m_buttonBoard, 5);
  private Trigger whiteTwo    = new JoystickButton(m_buttonBoard, 6);
  private Trigger redTwo      = new JoystickButton(m_buttonBoard, 7);
  private Trigger yellowTwo   = new JoystickButton(m_buttonBoard, 8);
  private Trigger greenTwo    = new JoystickButton(m_buttonBoard, 9);  
  private Trigger blueTwo     = new JoystickButton(m_buttonBoard, 10);

  // public static Pose3d targetPose = LimelightHelpers.getTargetPose3d_RobotSpace("");
  //public static Pose3d getTargetPose3d_RobotSpace();  
  private boolean isHighGear = false;
  private boolean isFieldRelative = false;
  public boolean yHeld = false;
  public boolean shooterRunning = false;
  boolean isInRange = false;

  public boolean hasUpdatedOdemetry = false;

  
  private Trigger isInRangeTrigger = new Trigger(()-> isInRange);
  
  private final Drivetrain m_swerve = new Drivetrain();
  private final Field2d m_field = new Field2d();

  private RawFiducial[] fiducials;
  private Pose2d limeLightPose;

  public double xPosition;
  public double yPosition;
  
  public double xPastPosition;
  public double yPastPosition;
  public double currentAngle;
  public double pastAngle;

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

  private intake m_intake = new intake(this);
  private shooterSubsystem m_ShooterSubsystem = new shooterSubsystem();
  private climbSubsystem m_climbSubsystem = new climbSubsystem();

  private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();
  
  // Slew rate limiters to make joystick inputs more gentle; Passing in "3" means 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter    = new SlewRateLimiter(3);

  private Command m_autonomousCommand;
  private Command m_autonomousCommand2;

  private String m_autoSelected;
  private final SendableChooser<String> m_AutoChooser = new SendableChooser<>();
  

  int id;                  // Tag ID
  double txnc;             // X offset (no crosshair)
  double tync;             // Y offset (no crosshair)
  double ta;               // Target area
  double distToCamera;     // Distance to camera
  double distToRobot;      // Distance to robot
  double ambiguity;        // Tag pose ambiguity
  int closestAprilTagID = 0;  //Tag ID with the greatest area

  //private double txToTurn = 0;
  private double angleToTurn = 0;

  public int[] m_focusAprilTags;

public Robot() {
  //CameraServer.startAutomaticCapture();
  
}

  @Override
  public void robotInit() {
    LimelightHelpers.setupPortForwardingUSB(0); //Port Forwarding for 2026 Limelight 3a
    scanForAprilTags();

    m_AutoChooser.setDefaultOption("None",      Constants.AutoConstants.kAutoProgram[0]);
    m_AutoChooser.addOption("Left Side Score",  Constants.AutoConstants.kAutoProgram[1]);
    m_AutoChooser.addOption("Center Score",     Constants.AutoConstants.kAutoProgram[2]);
    m_AutoChooser.addOption("Right Side Score", Constants.AutoConstants.kAutoProgram[3]);
    m_AutoChooser.addOption("Cen Score, Climb", Constants.AutoConstants.kAutoProgram[4]);
    m_AutoChooser.addOption("Right Side Shooter and Move", Constants.AutoConstants.kAutoProgram[5]); 
    m_AutoChooser.addOption("Left Side Shooter and Move", Constants.AutoConstants.kAutoProgram[6]);
    m_AutoChooser.addOption("Left Side Shooter and Move To Depo", Constants.AutoConstants.kAutoProgram[7]);
    
    SmartDashboard.putData("Auto Choices", m_AutoChooser);  //Sync the Autochooser


    backButton.onTrue(shiftGears()); 
    startButton.onTrue(changeIsFieldRelative());
    
    xButton.onTrue(turnTowardAprilTag(Constants.AprilTagConstants.middleIds));
    yButton.onTrue(new InstantCommand(() -> yButtonHeld(true)))
      .onFalse(new InstantCommand(() -> yButtonHeld(false)));

    aButton.onTrue(turnToAGlobalDirection(0));
    
    //yButton.onTrue(new setShooterSpeed(Constants.c_defaultShooterSpeed, m_ShooterSubsystem))
    //        .onFalse(new setShooterSpeed(0, m_ShooterSubsystem));  //Just for testing
    
    //lbButton.onTrue(new setShooterSpeed(Constants.c_defaultShooterSpeed, m_ShooterSubsystem))
    //        .onFalse(new setShooterSpeed(Constants.c_shooterMotorStop, m_ShooterSubsystem));


    povUp.onTrue(new InstantCommand(() -> m_ShooterSubsystem.incShooterASpeedOffset()));
    povDown.onTrue(new InstantCommand(() -> m_ShooterSubsystem.decShooterASpeedOffset()));
    povLeft.onTrue(new InstantCommand(() -> m_intake.incIntakeSlideOffset()));
    povRight.onTrue(new InstantCommand(() -> m_intake.decIntakeSlideOffset()));

    lbButton.onTrue(new extendIntake(true, m_intake))
                .onFalse(new extendIntake(false, m_intake));

    //  //extend
    redOne.onTrue(new intakeFuelCmd(-Constants.c_defaultIntakeSpeed, m_intake));
      redOne.onFalse(new intakeFuelCmd(0, m_intake));
     yellowOne.onTrue(new setShooterSpeed(Constants.c_defaultShooterSpeed, m_ShooterSubsystem, this, true))
              .onFalse(new setShooterSpeed(Constants.c_shooterMotorStop, m_ShooterSubsystem, this, false));
    blueOne.onTrue(new setShooterSpeed(m_ShooterSubsystem, true, this, true)) 
               .onFalse(new setShooterSpeed(Constants.c_shooterMotorStop, m_ShooterSubsystem, this, false));
    greenOne.onTrue(new climberCommand(climbLevel.e_levelOne, m_climbSubsystem));  
    //blueOne.onTrue(new climberCommand(climbLevel.e_levelTwo, m_climbSubsystem));
    // blueOne.onTrue(new setClimbSpeed(0.6, m_climbSubsystem));
   
    rbButton.onTrue(new retractIntake(true, m_intake, Constants.kSlideSpeed));
    rbButton.onFalse(new retractIntake(false, m_intake, Constants.kSlideSpeed));    //retract
    redTwo.onTrue(new intakeFuelCmd(Constants.c_defaultIntakeSpeed, m_intake))
            .onFalse(new intakeFuelCmd(0, m_intake));

          
    blueTwo.onTrue(new setShooterSpeed(-Constants.c_defaultShooterSpeed, m_ShooterSubsystem, this, false))
              .onFalse(new setShooterSpeed(Constants.c_shooterMotorStop, m_ShooterSubsystem, this, false));
    greenTwo.onTrue(new climberCommand(climbLevel.e_floor, m_climbSubsystem));  

    whiteOne.onTrue(new retractAndIntake(true, false, m_intake, Constants.kSlideSpeedSlow, -Constants.c_defaultIntakeSpeed));
    whiteOne.onFalse(new retractAndIntake(false, false, m_intake, 0, 0));

    whiteTwo.onTrue(new extendIntake(true, m_intake))
                .onFalse(new extendIntake(false, m_intake));


    
    //whiteOne.onTrue(new intakeFuelCmd(-Constants.c_defaultIntakeSpeed, m_intake));
    //whiteOne.onFalse(new intakeFuelCmd(0, m_intake));

    //blueTwo.onTrue(new climberCommand(climbLevel.e_levelOne, m_climbSubsystem));
    // blueTwo.onTrue(new setClimbSpeed(-0.6, m_climbSubsystem));

    limeLightPose = new Pose2d();
  } 


  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putData("Auto Choices", m_AutoChooser);  //Sync the Autochooser
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    
    m_swerve.updateOdometry();

    publisher.set(m_swerve.m_odometry.getEstimatedPosition());
    
    //isInRangeTrigger.whileTrue(m_LedSubsystem.runInRange());

    scanForAprilTags();
    
    // GetAngleChange();
  }

  

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    SmartDashboard.putData("Field", m_field);

    m_autoSelected = m_AutoChooser.getSelected();
    m_autonomousCommand = getAutonomousCommand();
    m_autonomousCommand2 = getAutonomousCommand2();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }    
    if(m_autonomousCommand2 != null){
      m_autonomousCommand2.schedule();
    }
  }

  @Override
  public void teleopInit() {
    // Do this in either robot or subsystem init
    SmartDashboard.putData("Field", m_field);
    publisher.set(m_swerve.m_odometry.getEstimatedPosition());
    
    // This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
   

  }
  
  @Override
  public void autonomousPeriodic() {
    //driveWithJoystick(false);
    publishToDashboard();
    m_swerve.publishToDashboard();

    // Do this in either robot periodic or subsystem periodic
    m_field.setRobotPose(m_swerve.m_odometry.getEstimatedPosition());
  }

  @Override
  public void testPeriodic() {
    m_swerve.publishToDashboard();    //publish the abs & adj. turn readings without the Drive function
  }

  @Override
  public void teleopPeriodic() {
    publishToDashboard();


    // Do this in either robot periodic or subsystem periodic
    m_field.setRobotPose(m_swerve.m_odometry.getEstimatedPosition());
 
    //Set the max speed constant to use high (regular) or low speed based on isHighGear
    double l_MaxSpeed     = isHighGear?Constants.kMaxRobotSpeed       :Constants.kMaxRobotSpeedLowGear;
    double l_MaxAngSpeed  = isHighGear?Constants.kMaxRobotAngularSpeed:Constants.kMaxRobotAngularSpeedLowGear;
    
    
    final var xSpeed =
      -m_xspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftY(), 0.1))
      * l_MaxSpeed;
    SmartDashboard.putNumber("xSpeed", xSpeed);

    final var ySpeed =
      -m_yspeedLimiter.calculate(MathUtil.applyDeadband(m_controller.getLeftX(), 0.1))
        * l_MaxSpeed;
    SmartDashboard.putNumber("ySpeed", ySpeed);

    final var rot =
      -m_rotLimiter.calculate(MathUtil.applyDeadband(m_controller.getRightX(), 0.1))
        * l_MaxAngSpeed;
    SmartDashboard.putNumber("rot", rot);

    m_swerve.drive(xSpeed, ySpeed, rot, isFieldRelative, getPeriod());  
    
    SmartDashboard.putNumber("distanceFromHubCenter", GetDistanceFromHubWithPosition());


    // //Position Tracking
    // xPosition = xPosition + (m_swerve.robotPose2d.getX() - xPastPosition);
    // yPosition = yPosition + (m_swerve.robotPose2d.getY() - yPastPosition);
    // AtemptToUpdateLocalPosition();
    // xPastPosition = m_swerve.robotPose2d.getX();
    // yPastPosition = m_swerve.robotPose2d.getY();
    RawFiducial[] fids = LimelightHelpers.getRawFiducials("");
    if(fids.length > 1){
      Boolean safeToAdd = true;

        for (RawFiducial rawFiducial : fids) {
                if(rawFiducial.ambiguity > .2){
                  safeToAdd = false;
                }
        };

      if(safeToAdd){
          limeLightPose = LimelightHelpers.getBotPose2d_wpiBlue("");
        m_swerve.m_odometry.addVisionMeasurement(limeLightPose, Timer.getFPGATimestamp());
        hasUpdatedOdemetry = true;
      }
      //m_swerve.resetOdometry(limeLightPose);
    }
    
    xPosition = limeLightPose.getX();
    yPosition = limeLightPose.getY();
    currentAngle = limeLightPose.getRotation().getRadians();
    SmartDashboard.putNumber("xPosition", xPosition);
    SmartDashboard.putNumber("yPosition", yPosition);
     SmartDashboard.putNumber("angle", currentAngle);
    

    // //AngleTracking
    // currentAngle = currentAngle + (m_swerve.robotPose2d.getRotation().getRadians() - pastAngle);
    // AtemptToUpdateLocalRotation();
    // pastAngle = m_swerve.robotPose2d.getRotation().getRadians();
  }

  
  public double GetAngleChangeToHub(){
    double distanceToHubBlue = Math.sqrt(Math.pow((xPosition-Constants.AprilTagConstants.blueHubLocation[0]),2) + Math.pow((yPosition-Constants.AprilTagConstants.blueHubLocation[1]),2));
    double distanceToHubRed = Math.sqrt(Math.pow((xPosition-Constants.AprilTagConstants.redHubLocation[0]),2) + Math.pow((yPosition-Constants.AprilTagConstants.redHubLocation[1]),2));

    double hubX = 0;
    double hubY = 0;
    if((distanceToHubBlue < distanceToHubRed)){
      hubX = Constants.AprilTagConstants.blueHubLocation[0];
      hubY = Constants.AprilTagConstants.blueHubLocation[1];
    }else{
      hubX = Constants.AprilTagConstants.redHubLocation[0];
      hubY = Constants.AprilTagConstants.redHubLocation[1];
    }
    double offsetPointX = xPosition + 2;

    double sideA = 2;
    double sideB = Math.sqrt(Math.pow(xPosition - hubX, 2) + Math.pow(yPosition - hubY, 2));
    double sideC = Math.sqrt(Math.pow(offsetPointX - hubX, 2) + Math.pow(yPosition - hubY, 2));

    return Math.acos((Math.pow(sideA, 2) + Math.pow(sideB, 2) - Math.pow(sideC, 2))/(sideA * sideB * 2));

  }
  public double GetDistanceFromHubWithPosition(){
     double distanceToHubBlue = Math.sqrt(Math.pow((xPosition-Constants.AprilTagConstants.blueHubLocation[0]),2) + Math.pow((yPosition-Constants.AprilTagConstants.blueHubLocation[1]),2));
    double distanceToHubRed = Math.sqrt(Math.pow((xPosition-Constants.AprilTagConstants.redHubLocation[0]),2) + Math.pow((yPosition-Constants.AprilTagConstants.redHubLocation[1]),2));

    double hubX = 0;
    double hubY = 0;
    if((distanceToHubBlue < distanceToHubRed)){
      hubX = Constants.AprilTagConstants.blueHubLocation[0];
      hubY = Constants.AprilTagConstants.blueHubLocation[1];
    }else{
      hubX = Constants.AprilTagConstants.redHubLocation[0];
      hubY = Constants.AprilTagConstants.redHubLocation[1];
    }

    return Math.sqrt(Math.pow(xPosition - hubX, 2) + Math.pow(yPosition - hubY, 2));
  }
  public double GetAngleChange(){
    //Getting Yaw
    double[] distanceAndId = getDistToTag(Constants.AprilTagConstants.middleIds);
    if (distanceAndId[0] == 0){
      return 0;
    }
    double distanceA = distanceAndId[0];
    double distanceB = .3556;
    double distanceC = 0; //logic is below

    distanceC =  getDistToTag(new int[] { Constants.AprilTagConstants.IDpairs.get((int)distanceAndId[1]) })[0];

    if (distanceC == 0){
      return 0;
    }
    double angle = (distanceA*distanceA + distanceB* distanceB - distanceC*distanceC)/(2 * distanceA * distanceB);
    angle = Math.toDegrees(Math.acos(angle));
   double aprilTagYaw = Math.toRadians(90-angle);


 double distanceFromTagToCenter = .5969;
 
    
    double currentRotation = Math.atan(Math.cos(aprilTagYaw)/Math.sin(aprilTagYaw));
    double desiredRotation = Math.atan((distanceA* Math.cos(aprilTagYaw) + distanceFromTagToCenter)/(distanceA * Math.sin(aprilTagYaw)));
    return desiredRotation - currentRotation;
  }
  public double getDistanceFromHubCenter(){
     //Getting Yaw
    double[] distanceAndId = getDistToTag(Constants.AprilTagConstants.middleIds);
    if (distanceAndId[0] == 0){
      return 0;
    }
    double distanceA = distanceAndId[0];
    double distanceB = .3556;
    double distanceC = 0; //logic is below

    distanceC =  getDistToTag(new int[] { Constants.AprilTagConstants.IDpairs.get((int)distanceAndId[1]) })[0];

    if (distanceC == 0){
      return 0;
    }
    double angle = (distanceA*distanceA + distanceB* distanceB - distanceC*distanceC)/(2 * distanceA * distanceB);
    angle = Math.toDegrees(Math.acos(angle));
   double aprilTagYaw = Math.toRadians(90-angle);


  double distanceFromTagToCenter = .5969;
  double distanceFinal = Math.sqrt(Math.pow((distanceA* Math.cos(aprilTagYaw) + distanceFromTagToCenter), 2) + Math.pow((distanceA * Math.sin(aprilTagYaw)),2));
    return distanceFinal;
  }
   public void yButtonHeld(boolean onOrOff){
     yHeld = onOrOff;
   }
  
  public void publishToDashboard()
  {
    SmartDashboard.putNumber("Controller Left X", m_controller.getLeftX());
    SmartDashboard.putNumber("Controller Left Y", m_controller.getLeftY());
    SmartDashboard.putNumber("Controller Right X", m_controller.getRightX());
    SmartDashboard.putNumber("Gyro Angle", m_swerve.m_gyro.getRotation2d().getDegrees());
    SmartDashboard.putBoolean("High Gear Enabled", isHighGear);
    SmartDashboard.putBoolean("isFieldRelative", isFieldRelative);
    SmartDashboard.putNumber("RightTrigger", m_controller.getRightTriggerAxis());   
    
    SmartDashboard.putNumber("RobotSpeed - x", m_swerve.getRobotChassisSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("RobotSpeed - y", m_swerve.getRobotChassisSpeeds().vyMetersPerSecond);
    SmartDashboard.putNumber("RobotSpeed - rot", m_swerve.getRobotChassisSpeeds().omegaRadiansPerSecond);
  }

  public void scanForAprilTags() {
    fiducials = LimelightHelpers.getRawFiducials("");
    closestAprilTagID = 0;  //reset the closest tag ID each time
    double closestAprilTagArea = 0;
    for (RawFiducial fiducial : fiducials) {

        id = fiducial.id;                    // Tag ID
        txnc = fiducial.txnc;             // X offset (no crosshair)
        tync = fiducial.tync;             // Y offset (no crosshair)
        ta = fiducial.ta;                 // Target area
        distToCamera = fiducial.distToCamera;  // Distance to camera
        distToRobot = fiducial.distToRobot;    // Distance to robot
        ambiguity = fiducial.ambiguity;   // Tag pose ambiguity
      
        if(ta > closestAprilTagArea) 
        {
          closestAprilTagArea = ta;
          closestAprilTagID = id;
        }

    }

    // if(fiducials.length>0)
    // {
    //   limeLightPose = LimelightHelpers.getBotPose2d_wpiBlue("");
    //   setOdoCommand(limeLightPose);  
    // }
    
        
    
    if( (distToCamera < Constants.AprilTagConstants.shootMaxRange[1]) && 
        (distToCamera > Constants.AprilTagConstants.shootMaxRange[0]) )   //just a test for the dist to hub 
    {
      System.out.println("is in range");
      isInRange = true;
      m_LedSubsystem.runInRange();
    }

    SmartDashboard.putBoolean("Is In Range?", isInRange);
    SmartDashboard.putNumber("distToCamera", distToCamera);
    SmartDashboard.putNumber("Txnc", txnc);
    SmartDashboard.putNumber("closestAprilTag", closestAprilTagID);
  }
    public double getAprilYaw (int[] tagIDs) {
    double yaw = 0; //Math.random(); //use random for simulation
  
    if (tagIDs.length != 0)
    {
      System.out.println("TagID != 0");
      for(RawFiducial fiducial : fiducials)   //cycle through all detected April Tags
      {
        for(int tagID : tagIDs) {             //determine if any of the detect tags are in the list of inters
          if (fiducial.id == tagID) {
            //yaw = fiducial.;
          } else {
            //txToTurn = 0;
            System.out.println("TagID = 0");
          }
        }
      }
    }
 
    return yaw;
  }


  public double getAprilTx (int[] tagIDs) {
    double txToTurn = 0; //Math.random(); //use random for simulation
  
    if (tagIDs.length != 0)
    {
      System.out.println("TagID != 0");
      for(RawFiducial fiducial : fiducials)   //cycle through all detected April Tags
      {
        for(int tagID : tagIDs) {             //determine if any of the detect tags are in the list of inters
          if (fiducial.id == tagID) {
            txToTurn = fiducial.txnc;
            System.out.println(txToTurn);
          } else {
            //txToTurn = 0;
            System.out.println("TagID = 0");
          }
        }
      }
    }

    angleToTurn = -(txToTurn)*(Math.PI/180); 
    SmartDashboard.putNumber("TXTesting", txToTurn);
    SmartDashboard.putNumber("AngleToTurn", angleToTurn);
    return angleToTurn;
  }

  public double[] getDistToTag (int[] tagIDs) {
    double distToTag = 0; //Math.random(); //use random for simulation
    double idUsed = 0;
    if (tagIDs.length != 0)
    {
      System.out.println("TagID != 0");
      for(RawFiducial fiducial : fiducials)   //cycle through all detected April Tags
      {
        for(int tagID : tagIDs) {             //determine if any of the detect tags are in the list of inters
          System.out.println(fiducial.id + ", " + tagID);
          if (fiducial.id == tagID) {
            distToTag = fiducial.distToRobot;
            idUsed = fiducial.id;
            System.out.println(distToTag);
          } else {
            //txToTurn = 0;
            System.out.println("TagID = 0");
          }
        }
      }
    }

    SmartDashboard.putNumber("distToTag", distToTag);

    //return distToTag;
    return new double[]{distToTag, idUsed}; //for sim only
  }

  public Command turnTowardAprilTag(int[] tagIDs) {
    m_focusAprilTags = tagIDs;
    return new turnTowardsAprilPID(tagIDs, getPeriod(), m_swerve, this);
  }

  /** AUTO Stuff below here **/
  public Command getAutonomousCommand() {

    Command temp = new Command() {};
    
    switch (m_autoSelected) {
      case "None":
        new InstantCommand(() -> setOdoCommand(Constants.AutoConstants.kStartingPoses[1]));
        break;

      case "ChoreoTestAuto":
        new InstantCommand(() -> setOdoCommand(Constants.AutoConstants.kStartingPoses[4]));
        break;

      case "LeftSideScore":
        temp = 
          Commands.sequence(
           new InstantCommand(() -> setOdoCommand(Constants.AutoConstants.kStartingPoses[1])),
            scoreAuto(),
           new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(1)
          );
        break;

      case "CenterScore":
        temp = 
          Commands.sequence(
            new InstantCommand(() -> setOdoCommand(Constants.AutoConstants.kStartingPoses[2])),
            driveStraight(-1.5),
            //new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(1),
            driveSpinways(-Math.PI/12),
            //new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(1),
            scoreAuto(),
            new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(1)
        );
        break;

      case "RightSideScore":
        temp =
          Commands.sequence(
            new InstantCommand(() -> setOdoCommand(Constants.AutoConstants.kStartingPoses[1])),         
            scoreAuto(),
            new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(1)
        );
        break;

      case "RightSideScoreAndMove":
        temp =
          Commands.sequence(
            new InstantCommand(() -> setOdoCommand(Constants.AutoConstants.kStartingPoses[1])),         
            scoreAuto(),
            new WaitCommand(6),
             moveAuto(false),
           new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(1)
        );
        break;
        case "LeftSideScoreAndMove":
          temp =
            Commands.sequence(
              new InstantCommand(() -> setOdoCommand(Constants.AutoConstants.kStartingPoses[1])),
              scoreAuto(),
              new WaitCommand(6),
               moveAuto(true),
           new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(1)
          
          );
          break;

      case "ShootCenterThenClimb":
        temp = Commands.sequence(
          new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
          driveStraight(-1),
          shootByDistAuto(5),
          driveSpinways(Math.PI),
          driveStraight(2.18), //86.6 inches
          climb(0.6),
          new InstantCommand(() -> System.out.println("Done !")));
        break;
      
      case "CrossToShooter":

        break;

      case "TrenchAuto":
          temp = Commands.sequence(
            //also do not mind... also not suspicious... the ellipses do not mean anything...
            new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))),
            scoreAuto(),
            new WaitCommand(6),
            

            new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(1)
          );
        break;
    
      default:
        break;
    }
    return temp;
  }
  public Command getAutonomousCommand2() {

    Command temp = new Command() {};
    
    switch (m_autoSelected) {
      case "None":
        temp = null;
        break;

      case "LeftSideScore":
        temp = shooter(3350);
        break;

      case "CenterScore":
        temp = shooter(3300);
        break;

      case "RightSideScore":
        temp =shooter(3350);
        break;

      case "ShootCenterThenClimb":
        temp = null;
        break;
      
      case "CrossToShooter":
        temp = null;
        break;
      
      case "RightSideScoreAndMove":
        temp = shooter(3350);
        break;
    case "LeftSideScoreAndMove":
        temp = shooter(3350);
        break;

      default:
        break;
    }
    return temp;
  }
 
  public InstantCommand resetOdoCommand() {
    return new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0))));
  }

  public InstantCommand setOdoCommand(Pose2d poseFromLimelight) {
    return new InstantCommand(() -> m_swerve.resetOdometry(poseFromLimelight));
  }

  public Command driveStraight(double dist) {
    return new driveStraightPID(dist, getPeriod(), m_swerve);
  }
    
  public Command driveSideways(double dist) {
    return new driveSidewaysPID(dist, getPeriod(), m_swerve);
  }

  public Command driveSpinways(double angle) {
    return new driveSpinwaysPID(angle, getPeriod(), m_swerve);
  }

  public Command driveToPosition(Pose2d position) {
    return new driveToPositionPID(position, getPeriod(), m_swerve);
  }


  public Command shiftGears() {
    return Commands.sequence(
        new InstantCommand(() -> System.out.println("shiftGears")),
        new InstantCommand(() -> isHighGear=!isHighGear)
    );
  }

  public Command shootBySpeedAuto(double speed) {
    return Commands.sequence(
            //new setShooterSpeed(m_ShooterSubsystem, true, this),
            shootFuel(speed), 
            new WaitCommand(9),
            stopShootFuel());
  }

  public Command shootByDistAuto(double shootTime) {
    return Commands.sequence(
            new setShooterSpeed(m_ShooterSubsystem, true, this, true),
            new WaitCommand(shootTime),
            new setShooterSpeed(Constants.c_shooterMotorStop, m_ShooterSubsystem, this, false)
    );
  }
  

  public Command shootFuel(double speed) {
    return new setShooterSpeed(speed, m_ShooterSubsystem, this, true);
  }

  public Command climb(double speed) {
    return new setClimbSpeed(speed, m_climbSubsystem);
  }

 public Command stopShootFuel() {
    return new setShooterSpeed(0, m_ShooterSubsystem, this, false);
  }

  public Command changeIsFieldRelative() {
    return Commands.sequence(
      new InstantCommand(() -> System.out.println("fieldRelative")),
        new InstantCommand(() -> isFieldRelative=!isFieldRelative)
    );
  }

  /* This function is the standard 'score in auto' function
   * Extends intake
   * Spins up Intake & REVERSE Shoot - the goal is to get all the fuel in the hopper
   * Shoot - 1st round
   * Retract Hopper to push fuel further towards the hopper
   */
  public Command shooter(double speed){
    return Commands.sequence(
      new WaitCommand(.25),
      shootBySpeedAuto(speed)
    );

  }
  public Command scoreAuto() {   
    return Commands.sequence(
      extendIntakeForXSeconds(.75),
      intakeForXSeconds(.75), //LaunchBallsBackIntoHopper
        printCmd("Start Intake"),
      new WaitCommand(2), //waitDorballsToBeShotOut
      retractIntakeForXSeconds(1),
      extendIntakeForXSeconds(.5)
    );
  }

  public Command moveAuto(Boolean reverse) {
    double turnValue = 0; //Can All be Changed to be relative (turn angle to 0 degrees)
    double turnValue2 = 0;

    double[] distanceAndId = getDistToTag(Constants.AprilTagConstants.middleIds);

    if (distanceAndId[0] == 0){
      if (reverse) {
      turnValue = -.75;
      } else {
        turnValue = .75;
      }
    }else{
      double distanceA = distanceAndId[0];
      double distanceB = .3556;
      double distanceC = 0; //logic is below
      distanceC =  getDistToTag(new int[] { Constants.AprilTagConstants.IDpairs.get((int)distanceAndId[1]) })[0];
      if (distanceC == 0){
        if (reverse) {
        turnValue = -.75;
        } else {
          turnValue = .75;
        }
      }
      double angle = (distanceA*distanceA + distanceB* distanceB - distanceC*distanceC)/(2 * distanceA * distanceB);
      angle = Math.toDegrees(Math.acos(angle));
      if(turnValue == 0){
        turnValue =  90 - (90 - MathUtil.angleModulus(angle));
        turnValue = Math.toRadians(turnValue);
      }
    }
    if (reverse) {
        turnValue2 = Math.PI/2;
      } else {
        turnValue2 = -Math.PI/2;
    }

    return Commands.sequence(
      new InstantCommand(() -> System.out.println("Starting Move Auto")),
      new driveSpinwaysPID(turnValue, 0.02, m_swerve),
      new driveStraightPID(2.5, 0.02, m_swerve),
       extendIntakeForXSeconds(.5),
      new driveSpinwaysPID(turnValue2, 0.02, m_swerve),
      intakeWhilstDriving(3),
      new driveSpinwaysPID(Math.PI, 0.02, m_swerve),
      intakeWhilstDriving(3),
      new driveSpinwaysPID(-turnValue2, 0.02, m_swerve),
      new driveStraightPID(2.5, 0.02, m_swerve),
      new driveSpinwaysPID(turnValue, 0.02, m_swerve),
      shootBySpeedAuto(3500),
      new InstantCommand(() -> System.out.println("Starting Move Auto"))
    );
  }

  public Command moveToNeutralZone(Boolean reverse) {
    // Will work on this at home, do not mind this, definitely a red herring, does not mean anything, not suspicious at all...
    double turnValue = 0; //Can All be Changed to be relative (turn angle to 0 degrees)
    double turnValue2 = 0;

    double[] distanceAndId = getDistToTag(Constants.AprilTagConstants.middleIds);

    if (distanceAndId[0] == 0){
      if (reverse) {
        turnValue = .75;
      } else {
        turnValue = -.75;
      }
    } else {
      double distanceA = distanceAndId[0];
      double distanceB = .3556;
      double distanceC = 0; //logic is below
      distanceC =  getDistToTag(new int[] { Constants.AprilTagConstants.IDpairs.get((int)distanceAndId[1]) })[0];
      if (distanceC == 0){
        if (reverse) {
        turnValue = .75;
        } else {
          turnValue = -.75;
        }
      }
      double angle = (distanceA*distanceA + distanceB* distanceB - distanceC*distanceC)/(2 * distanceA * distanceB);
      angle = Math.toDegrees(Math.acos(angle));
      if(turnValue == 0){
        turnValue =  90 - (90 - MathUtil.angleModulus(angle));
        turnValue = Math.toRadians(turnValue);
      }
    }
    if (reverse) {
        turnValue2 = -Math.PI/2;
      } else {
        turnValue2 = Math.PI/2;
    }

    return Commands.sequence(
      new driveSpinwaysPID(turnValue, 0.02, m_swerve),
      new driveStraightPID(-2.5, 0.02, m_swerve),
       extendIntakeForXSeconds(.5),
      new driveSpinwaysPID(turnValue2, 0.02, m_swerve),
      intakeWhilstDriving(3)
    );
  }

  public Command extendIntakeForXSeconds(double seconds){
    return Commands.sequence(
      new extendIntake(true, m_intake),
      new InstantCommand(() -> System.out.println("Extending Intake Auto")),
      new WaitCommand(seconds),
      new extendIntake(false, m_intake)
    );
  }
  public Command retractIntakeForXSeconds(double seconds){
    return Commands.sequence(
      new retractIntake(true, m_intake, Constants.kSlideSpeedSlow),
      new WaitCommand(seconds),
      new retractIntake(false, m_intake, 0)

    );
  }
  public Command intakeForXSeconds(double seconds){
    return Commands.sequence(
       new intakeFuelCmd(-Constants.c_defaultIntakeSpeed, m_intake),
       new WaitCommand(seconds),
      new intakeFuelCmd(0, m_intake)
    );
  }
  public Command intakeWhilstDriving(double distance){
    return Commands.sequence(
      new intakeFuelCmd(-Constants.c_defaultIntakeSpeed, m_intake),
      new WaitCommand(.2),
      new driveStraightPID(distance, 0.02, m_swerve),
      new intakeFuelCmd(0, m_intake)
    );
  }
  public Command turnToAGlobalDirection(double direction){
    double tx =  direction - currentAngle;
    return new driveSpinwaysPID(tx, .02, m_swerve); 
  }
  public Command driveBackTillLimeLightSeen(){
    return new driveBackTillPositioned(-100, getPeriod(), m_swerve, this);
  }
  // public Command goToFieldPosition(double x, double y){
    
  //   driveStraight();
  //   driveSideways();
  // }
  public Command printCmd(String cmdName) {
    return new InstantCommand(() -> System.out.println(cmdName));
  }

}