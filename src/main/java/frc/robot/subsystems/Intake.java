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


public class intake extends SubsystemBase {
  
  /* Intake Subsystem Constants */
  public static final int k_tiltMotorPortA = 30;
  public static final int k_tiltMotorPortB = 31;
  public static final int k_intakeMotorPortA = 32;
  public static final int k_intakeMotorPortB = 33;
  public static final double k_tiltEncOffsetA = 0;
  public static final double k_tiltEncOffsetB = 0;
  public static final double[] k_tiltDistanceSetpointA = {
    0,                        // Up
    1                         // Down / Floor
  };
  public static final double[] k_tiltDistanceSetpointB = {
    0,                        // Up
    1                         // Down / Floor
  };

  public static final double KpTilt = 0.5;
  public static final double tiltMaxVelocity = Math.PI;
  public static final double tiltMaxAccel = Math.PI;
  
  /* Intake Subsystem Variables / Objects */
  private SparkMax tiltMotorA;  
  private SparkMax tiltMotorB;
  private SparkMax intakeMotorA;
  private SparkMax intakeMotorB;
  
  private RelativeEncoder m_tiltEncoderA;
  private RelativeEncoder m_tiltEncoderB;
  
  private double m_desiredIntakeSpeed = 0;
  private double m_actualIntakeSpeed = 0;
  private static double c_maxIntakeSpeed = 1;

  private double desiredtiltDistanceA = k_tiltDistanceSetpointA[0];
  private double desiredtiltDistanceB = k_tiltDistanceSetpointB[0];
  private double actualTiltAngleA = 0;
  private double actualTiltAngleB = 0;
  private double TiltAngleOffsetA = 0;
  private double TiltAngleOffsetB = 0;
  
  private final ProfiledPIDController m_intakePID;
  private final ProfiledPIDController m_tiltPIDControllerA;
  private final ProfiledPIDController m_tiltPIDControllerB;
      
  

  private double desiredIntakeSpeed = 0;

  public intake() {

    intakeMotorA = new SparkMax(k_intakeMotorPortA, SparkLowLevel.MotorType.kBrushless);
    intakeMotorB = new SparkMax(k_intakeMotorPortB, SparkLowLevel.MotorType.kBrushless);
    tiltMotorA = new SparkMax(k_tiltMotorPortA, SparkLowLevel.MotorType.kBrushless);
    tiltMotorB = new SparkMax(k_tiltMotorPortB, SparkLowLevel.MotorType.kBrushless);
    m_tiltEncoderA = tiltMotorA.getEncoder();
    m_tiltEncoderB = tiltMotorA.getEncoder();

    
    m_intakePID =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              c_maxIntakeSpeed,
              5*c_maxIntakeSpeed));
    m_intakePID.setTolerance(1);  //sets the tolerance for the PID controller, in meters


    m_tiltPIDControllerA =
      new ProfiledPIDController(
          KpTilt,
          0,
          0,
          new TrapezoidProfile.Constraints(
              tiltMaxVelocity,
              tiltMaxAccel));
    m_tiltPIDControllerA.setTolerance(.05);  //sets the tolerance for the PID controller, in meters

    m_tiltPIDControllerB =
      new ProfiledPIDController(
          KpTilt,
          0,
          0,
          new TrapezoidProfile.Constraints(
              tiltMaxVelocity,
              tiltMaxAccel));
    m_tiltPIDControllerB.setTolerance(.05);  //sets the tolerance for the PID controller, in meters
    
    // Set the default command for a subsystem here. (set the claw speed to 0)
    //setDefaultCommand(new setClawSpeed(0, this));
  }

  @Override
  public void periodic() {
    actualTiltAngleA = (m_tiltEncoderA.getPosition()-k_tiltEncOffsetA)*2*Math.PI;
    actualTiltAngleB = (m_tiltEncoderB.getPosition()-k_tiltEncOffsetB)*2*Math.PI;

    m_actualIntakeSpeed = intakeMotorA.get();
    

    //Publish Algae Grabber STuff to the Dashboard
    SmartDashboard.putNumber("TiltAngleA", actualTiltAngleA);
    SmartDashboard.putNumber("TiltAngleB", actualTiltAngleB);
    SmartDashboard.putNumber("IntakeSpeed", desiredIntakeSpeed);
    SmartDashboard.putNumber("TiltAngleOffsetA", TiltAngleOffsetA);
    SmartDashboard.putNumber("TiltAngleOffsetB", TiltAngleOffsetB);
    SmartDashboard.putNumber("Desired Tilt Angle A", desiredtiltDistanceA);
    SmartDashboard.putNumber("Desired Tilt Angle B", desiredtiltDistanceB);
    SmartDashboard.putNumber("tiltMotorACurrent", tiltMotorA.getOutputCurrent());
    SmartDashboard.putNumber("tiltMotorBCurrent", tiltMotorB.getOutputCurrent());
    SmartDashboard.putNumber("TiltPID_ErrorA", m_tiltPIDControllerA.getPositionError());
    SmartDashboard.putNumber("TiltPID_ErrorB", m_tiltPIDControllerB.getPositionError());    

    intakeMotorA.set(m_intakePID.calculate(m_actualIntakeSpeed, m_desiredIntakeSpeed));
    intakeMotorB.set(m_intakePID.calculate(m_actualIntakeSpeed, m_desiredIntakeSpeed));


  }

  public void setDesiredTiltAngleA(double angle) {
    desiredtiltDistanceA = angle + TiltAngleOffsetA;
  }
  
    public void setDesiredTiltAngleB(double angle) {
    desiredtiltDistanceB= angle + TiltAngleOffsetB;
  }


    public void extend() {
      setDesiredTiltAngleA(k_tiltDistanceSetpointA[1]);
      setDesiredTiltAngleB(k_tiltDistanceSetpointB[1]);
    }

    public void retract() {
      setDesiredTiltAngleA(k_tiltDistanceSetpointA[0]);
      setDesiredTiltAngleB(k_tiltDistanceSetpointB[0]);
    }

    public void setIntakeSpeed(double speed) {
      m_desiredIntakeSpeed = speed;
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
