package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.motorDiagnostics;
import frc.robot.Constants;
import frc.robot.Robot;


public class intake extends SubsystemBase {
  
  private Robot m_robot;
  /* Intake Subsystem Constants */
  public static final int k_slideMotorPortA = 26;
  public static final int k_intakeMotorPortA = 32;
  
  public static final double Kpslide = 0.5;
  public static final double slideMaxVelocity = Math.PI;
  public static final double slideMaxAccel = Math.PI;
  
  private static final double c_slideOffset = .05;

  /* Intake Subsystem Variables / Objects */
  private SparkMax slideMotorA;  
  private SparkMax intakeMotorA;

  private motorDiagnostics intakeDiags;
  private motorDiagnostics slideDiags;
  
  private double m_desiredIntakeSpeed = 0;
  private double m_actualIntakeSpeed = 0;

  private double m_desiredSlideSpeed = 0;
  
  private final ProfiledPIDController m_slidePIDControllerA;
  
  private RelativeEncoder m_slideEncoderA;
  private double m_slidePosition;
  private double m_slideOffset = 0;
  private boolean m_slideFullyRetracted = true;
  private boolean m_slideFullyExtended = false;

  private final double c_slideStowPos = 5;
  private final double c_slideOutPos = 33;
  private final double c_slideShootOutPos = 25;
   

  public intake(Robot robot) {
    m_robot = robot;
    intakeMotorA = new SparkMax(k_intakeMotorPortA, SparkLowLevel.MotorType.kBrushless);
    intakeDiags = new motorDiagnostics(intakeMotorA, "Intake Motor Diag");
    
    slideMotorA = new SparkMax(k_slideMotorPortA, SparkLowLevel.MotorType.kBrushless);
    slideDiags = new motorDiagnostics(slideMotorA, "Slide Motor Diag");  
    m_slideEncoderA = slideMotorA.getEncoder();  


    m_slidePIDControllerA =
      new ProfiledPIDController(
          Kpslide,
          0,
          0,
          new TrapezoidProfile.Constraints(
              slideMaxVelocity,
              slideMaxAccel));
    m_slidePIDControllerA.setTolerance(.05);  //sets the tolerance for the PID controller, in meters

   
  }

  @Override
  public void periodic() {
    m_slidePosition = m_slideEncoderA.getPosition();    //gets raw position from Slide Encoder    
    m_actualIntakeSpeed = intakeMotorA.get();
    SmartDashboard.putNumber("slidePercent", Math.round(m_slidePosition/c_slideOutPos * 100));


    m_slideFullyRetracted = (m_slidePosition <= c_slideStowPos );
    m_slideFullyExtended = (m_slidePosition >= c_slideOutPos);

    //Publish Stuff to the Dashboard
    SmartDashboard.putNumber("IntakeSpeed", m_desiredIntakeSpeed);
    SmartDashboard.putNumber("Slide Speed", m_actualIntakeSpeed);
    SmartDashboard.putNumber("Desired Slide Speed", m_desiredSlideSpeed);
    SmartDashboard.putNumber("slidePosition", m_slidePosition);
    SmartDashboard.putNumber("slidePosOffset", m_slideOffset);
    SmartDashboard.putBoolean("SlideFullyIn?", m_slideFullyRetracted);
    SmartDashboard.putBoolean("SlideFullyOut?", m_slideFullyExtended);
    
    intakeDiags.publishMotorData();
    slideDiags.publishMotorData();
    
   

    intakeMotorA.set(m_desiredIntakeSpeed); 
    // if (m_robot.shooterRunning && m_slidePosition > c_slideShootOutPos){
    //     m_desiredSlideSpeed = -.25; 
    //   }     
    

    if(((m_slideFullyRetracted && m_desiredSlideSpeed < 0) || (m_slideFullyExtended && m_desiredSlideSpeed > 0)) && (!m_robot.yHeld)){
      slideMotorA.set(0);
    }else{
      slideMotorA.set(m_desiredSlideSpeed);
            
    }
    if(m_robot.yHeld){
     m_slideEncoderA.setPosition(0);
    }
  }

  public void incIntakeSlideOffset() {
    m_slideOffset = m_slideOffset + c_slideOffset;
  }

  public void decIntakeSlideOffset() {
    m_slideOffset = m_slideOffset - c_slideOffset;
  }

  public void setIntakeSpeed(double speed) {
    m_desiredIntakeSpeed = speed;
  }

  public void setDesiredSlideSpeed(double speed) {
    m_desiredSlideSpeed = speed;
  }

  public double getIntakeSpeed() {
    return m_actualIntakeSpeed;
  }
    
  public void initDefaultCommand() {
    // When Idle, set the speeds to zero            
    Command initSequence = Commands.sequence(
      new InstantCommand(() -> setIntakeSpeed(0)));
      
    initSequence.addRequirements(this);
    setDefaultCommand(initSequence);      
  }

}
