// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
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
import frc.robot.commands.retractIntake;
import frc.robot.commands.setClimbSpeed;
import frc.robot.commands.setShooterSpeed;
import frc.robot.commands.turnTowardsAprilPID;
import frc.robot.commands.climberCommand.climbLevel;
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
  
  private boolean isHighGear = false;
  private boolean isFieldRelative = false;
  boolean isInRange = false;

  private Trigger isInRangeTrigger = new Trigger(()-> isInRange);
  
  private final Drivetrain m_swerve = new Drivetrain();
  private final Field2d m_field = new Field2d();

  private RawFiducial[] fiducials;

  StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault().getStructTopic("MyPose", Pose2d.struct).publish();

  private intake m_intake = new intake();
  private shooterSubsystem m_ShooterSubsystem = new shooterSubsystem();
  private climbSubsystem m_climbSubsystem = new climbSubsystem();

  private final LEDSubsystem m_LedSubsystem = new LEDSubsystem();
  
  // Slew rate limiters to make joystick inputs more gentle; Passing in "3" means 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter    = new SlewRateLimiter(3);

  private Command m_autonomousCommand;

  private String m_autoSelected;
  private final SendableChooser<String> m_AutoChooser = new SendableChooser<>();
  private final SendableChooser<String> m_AprilTagSelected = new SendableChooser<>();

  int id;                  // Tag ID
  double txnc;             // X offset (no crosshair)
  double tync;             // Y offset (no crosshair)
  double ta;               // Target area
  double distToCamera;     // Distance to camera
  double distToRobot;      // Distance to robot
  double ambiguity;        // Tag pose ambiguity
  int closestAprilTagID = 0;  //Tag ID with the greatest area

  private double txToTurn = 0;
  private double angleToTurn = 0;

  public int[] m_focusAprilTags;

public Robot() {
  //CameraServer.startAutomaticCapture();
  
}

  @Override
  public void robotInit() {
    LimelightHelpers.setupPortForwardingUSB(0); //Port Forwarding for 2026 Limelight 3a

    m_AutoChooser.setDefaultOption("None", Constants.AutoConstants.kAutoProgram[0]);
    m_AutoChooser.addOption("Auto 1", Constants.AutoConstants.kAutoProgram[1]);
    m_AutoChooser.addOption("Auto 2", Constants.AutoConstants.kAutoProgram[2]);
    m_AutoChooser.addOption("Auto 3", Constants.AutoConstants.kAutoProgram[3]);
    SmartDashboard.putData("Auto Choices", m_AutoChooser);  //Sync the Autochooser

    m_AprilTagSelected.setDefaultOption("None", "None");
    m_AprilTagSelected.addOption("1", "1");
    m_AprilTagSelected.addOption("2","2");
    m_AprilTagSelected.addOption("3","3");
  
    //m_ShooterSubsystem.initDefaultCommand();
    //m_intake.initDefaultCommand();

    backButton.onTrue(shiftGears()); 
    startButton.onTrue(changeIsFieldRelative());
    
    aButton.onTrue(turnTowardAprilTag(Constants.AprilTagConstants.frontTagsMiddle));
    yButton.onTrue(turnTowardAprilTag(Constants.AprilTagConstants.frontTagsSide)); 
    bButton.onTrue(turnTowardAprilTag(Constants.AprilTagConstants.rightTags));
    xButton.onTrue(turnTowardAprilTag(Constants.AprilTagConstants.leftTags));
    
    //yButton.onTrue(new setShooterSpeed(Constants.c_defaultShooterSpeed, m_ShooterSubsystem))
    //        .onFalse(new setShooterSpeed(0, m_ShooterSubsystem));  //Just for testing
    
    //lbButton.onTrue(new setShooterSpeed(Constants.c_defaultShooterSpeed, m_ShooterSubsystem))
    //        .onFalse(new setShooterSpeed(Constants.c_shooterMotorStop, m_ShooterSubsystem));


    povUp.onTrue(new InstantCommand(() -> m_ShooterSubsystem.incShooterASpeedOffset()));
    povDown.onTrue(new InstantCommand(() -> m_ShooterSubsystem.decShooterASpeedOffset()));
    povLeft.onTrue(new InstantCommand(() -> m_intake.incIntakeSlideOffset()));
    povRight.onTrue(new InstantCommand(() -> m_intake.decIntakeSlideOffset()));

    whiteOne.onTrue(new extendIntake(true, m_intake))
                .onFalse(new extendIntake(false, m_intake));

     //extend
    redOne.onTrue(new intakeFuelCmd(-Constants.c_defaultIntakeSpeed, m_intake));
    redOne.onFalse(new intakeFuelCmd(0, m_intake));
    //yellowOne.onTrue(new setShooterSpeed(-Constants.c_defaultShooterSpeed, m_ShooterSubsystem))
              //.onFalse(new setShooterSpeed(Constants.c_shooterMotorStop, m_ShooterSubsystem));
    yellowOne.onTrue(new setShooterSpeed(m_ShooterSubsystem, true, distToCamera))
              .onFalse(new setShooterSpeed(Constants.c_shooterMotorStop, m_ShooterSubsystem));
    greenOne.onTrue(new climberCommand(climbLevel.e_levelOne, m_climbSubsystem));  
    //blueOne.onTrue(new climberCommand(climbLevel.e_levelTwo, m_climbSubsystem));
    blueOne.onTrue(new setClimbSpeed(0.6, m_climbSubsystem));
   
    whiteTwo.onTrue(new retractIntake(true, m_intake));
    whiteTwo.onFalse(new retractIntake(false, m_intake));    //retract
    redTwo.onTrue(new intakeFuelCmd(Constants.c_defaultIntakeSpeed, m_intake))
            .onFalse(new intakeFuelCmd(0, m_intake));

          
    yellowTwo.onTrue(new setShooterSpeed(Constants.c_defaultShooterSpeed, m_ShooterSubsystem))
              .onFalse(new setShooterSpeed(Constants.c_shooterMotorStop, m_ShooterSubsystem));
    greenTwo.onTrue(new climberCommand(climbLevel.e_floor, m_climbSubsystem));  
    //blueTwo.onTrue(new climberCommand(climbLevel.e_levelOne, m_climbSubsystem));
    blueTwo.onTrue(new setClimbSpeed(-0.6, m_climbSubsystem));
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

    publisher.set(m_swerve.m_odometry.getPoseMeters());
    
    //isInRangeTrigger.whileTrue(m_LedSubsystem.runInRange());

    scanForAprilTags();
    
  }

  

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    SmartDashboard.putData("Field", m_field);

    m_autoSelected = m_AutoChooser.getSelected();
    m_autonomousCommand = getAutonomousCommand();
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }    
  }

  @Override
  public void teleopInit() {
    // Do this in either robot or subsystem init
    SmartDashboard.putData("Field", m_field);
    publisher.set(m_swerve.m_odometry.getPoseMeters());
    
    // This makes sure that the autonomous stops running when teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    /* Button Triggers */
    

  }
  
  @Override
  public void autonomousPeriodic() {
    //driveWithJoystick(false);
    publishToDashboard();
    m_swerve.publishToDashboard();
    m_swerve.updateOdometry();

    // Do this in either robot periodic or subsystem periodic
    m_field.setRobotPose(m_swerve.m_odometry.getPoseMeters());
  }

  @Override
  public void testPeriodic() {
    m_swerve.publishToDashboard();    //publish the abs & adj. turn readings without the Drive function
  }

  @Override
  public void teleopPeriodic() {
    publishToDashboard();
    //m_swerve.publishToDashboard();
    //switchGears(false);
    m_swerve.updateOdometry();

    // Do this in either robot periodic or subsystem periodic
    m_field.setRobotPose(m_swerve.m_odometry.getPoseMeters());
 
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
  }


  /* AUTO Stuff below here */
  public Command getAutonomousCommand() {

      Command temp = new Command() {};
    // Grabs the choser Auto from Shuffleboard
    switch (m_autoSelected) {
      case "None":
      //temp = m_swerve.getPathPlannerCommand();
        break;
      case "Auto 1":
        temp = Commands.sequence(
          new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),          
          driveStraight(1),
          new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(1));
        break;
      case "Auto 2":
        temp = Commands.sequence(
          new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
          new InstantCommand(() -> System.out.println("Command 1:")),
          driveStraight(1),
          new InstantCommand(() -> System.out.println("Stop & wait  .5 seconds")),
          new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(.5),
          new InstantCommand(() -> System.out.println("Done !")));
        break;
      case "Auto 3":
        temp = Commands.sequence(
        new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
        new InstantCommand(() -> System.out.println("Command 1:")),
        driveToPosition(new Pose2d(0, 0, new Rotation2d(Math.PI/2))),
        new InstantCommand(() -> System.out.println("Stop & wait  .5 seconds")),
        new InstantCommand(() -> m_swerve.drive(0,0,0,false, getPeriod())).repeatedly().withTimeout(.5),
        new InstantCommand(() -> System.out.println("Done !")));
        break;
      case "Shoot From Side":
        temp = Commands.sequence(
          new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
          new InstantCommand(() -> System.out.println("Shoot")),
          shootFuel(Constants.c_defaultShooterSpeed), 
          new WaitCommand(10),
          stopShootFuel(),
          new InstantCommand(() -> System.out.println("Done !")));
        break;
        
case "Shoot From Center":
        temp = Commands.sequence(
          new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
          ShootAuto(5000), 
          new InstantCommand(() -> System.out.println("Done !")));
        break;

case "Shoot Then Climb":
        temp = Commands.sequence(
          new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0)))),
          ShootAuto(5000),
          driveStraight(2.18), //86.6 inches
          climb(0.6),
          new InstantCommand(() -> System.out.println("Done !")));
        break;

      default:
        break;
    }
    return temp;
  }
 
  public InstantCommand resetOdoCommand() {
    return new InstantCommand(() -> m_swerve.resetOdometry(new Pose2d(0,0, new Rotation2d(0))));
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
public Command ShootAuto(double speed) {
  return Commands.sequence(
          new InstantCommand(() -> System.out.println("Shoot")),
          shootFuel(speed), 
          new WaitCommand(10),
          stopShootFuel());
}
  

  public Command shootFuel(double speed) {
    return new setShooterSpeed(speed, m_ShooterSubsystem);
  }

public Command climb(double speed) {
  return new setClimbSpeed(speed, m_climbSubsystem);
}

 public Command stopShootFuel() {
    return new setShooterSpeed(0, m_ShooterSubsystem);
  }
  public Command changeIsFieldRelative() {
    return Commands.sequence(
      new InstantCommand(() -> System.out.println("fieldRelative")),
        new InstantCommand(() -> isFieldRelative=!isFieldRelative)
    );
  }

  public Command turnTowardAprilTag(int[] tagIDs) {
    m_focusAprilTags = tagIDs;
    return new turnTowardsAprilPID(tagIDs, getPeriod(), m_swerve, this);
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

  public double getDistToTag (int[] tagIDs) {
    double distToTag = 0; //Math.random(); //use random for simulation
    if (tagIDs.length != 0)
    {
      System.out.println("TagID != 0");
      for(RawFiducial fiducial : fiducials)   //cycle through all detected April Tags
      {
        for(int tagID : tagIDs) {             //determine if any of the detect tags are in the list of inters
          if (fiducial.id == tagID) {
            distToTag = fiducial.distToCamera;
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
    return 0.82; //for sim only
  }

}