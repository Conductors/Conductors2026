package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class intake extends SubsystemBase {
  
  /* Intake Subsystem Constants */
  public static final int k_slideMotorPortA = 30;
  //public static final int k_slideMotorPortB = 31;
  public static final int k_intakeMotorPortA = 32;
  //public static final int k_intakeMotorPortB = 33;
  public static final double k_slideEncOffsetA = 0;
  //public static final double k_slideEncOffsetB = 0;
  /*public static final double[] k_slideDistanceSetpointA = {
    0,                        // Up
    1                         // Down / Floor
  };
  public static final double[] k_slideDistanceSetpointB = {
    0,                        // Up
    1                         // Down / Floor
  };*/


  public static final double Kpslide = 0.5;
  public static final double slideMaxVelocity = Math.PI;
  public static final double slideMaxAccel = Math.PI;
  
  private static final double c_slideOffset = .05;

  /* Intake Subsystem Variables / Objects */
  private SparkMax slideMotorA;  
  //private SparkMax slideMotorB;
  private SparkMax intakeMotorA;
  //private SparkMax intakeMotorB;
  
  private RelativeEncoder m_slideEncoderA;
  //private RelativeEncoder m_slideEncoderB;
  
  private double m_desiredIntakeSpeed = 0;
  private double m_actualIntakeSpeed = 0;
  private static double c_maxIntakeSpeed = 1;
  private final SimpleMotorFeedforward m_intakeFeedForward = new SimpleMotorFeedforward(0.5,0);

  //private double desiredslideDistanceA = k_slideDistanceSetpointA[0];
  //private double desiredslideDistanceB = k_slideDistanceSetpointB[0];
  private double actualslideAngleA = 0;
  //private double actualslideAngleB = 0;
  private double slideAngleOffsetA = 0;
  //private double slideAngleOffsetB = 0;

  private double m_desiredSlideSpeed = 0;
  
  private final ProfiledPIDController m_intakePID;
  private final ProfiledPIDController m_slidePIDControllerA;
  //private final ProfiledPIDController m_slidePIDControllerB;


      
  

  public intake() {

    intakeMotorA = new SparkMax(k_intakeMotorPortA, SparkLowLevel.MotorType.kBrushless);
    //intakeMotorB = new SparkMax(k_intakeMotorPortB, SparkLowLevel.MotorType.kBrushless);
    slideMotorA = new SparkMax(k_slideMotorPortA, SparkLowLevel.MotorType.kBrushless);
    //slideMotorB = new SparkMax(k_slideMotorPortB, SparkLowLevel.MotorType.kBrushless);
    m_slideEncoderA = slideMotorA.getEncoder();
    //m_slideEncoderB = slideMotorA.getEncoder();

    
    m_intakePID =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
              c_maxIntakeSpeed,
              5*c_maxIntakeSpeed));
    m_intakePID.setTolerance(1);  //sets the tolerance for the PID controller, in meters


    m_slidePIDControllerA =
      new ProfiledPIDController(
          Kpslide,
          0,
          0,
          new TrapezoidProfile.Constraints(
              slideMaxVelocity,
              slideMaxAccel));
    m_slidePIDControllerA.setTolerance(.05);  //sets the tolerance for the PID controller, in meters

    /*m_slidePIDControllerB =
      new ProfiledPIDController(
          Kpslide,
          0,
          0,
          new TrapezoidProfile.Constraints(
              slideMaxVelocity,
              slideMaxAccel));
    m_slidePIDControllerB.setTolerance(.05);  //sets the tolerance for the PID controller, in meters
    */
    // Set the default command for a subsystem here. (set the claw speed to 0)
    //setDefaultCommand(new setClawSpeed(0, this));
  }

  @Override
  public void periodic() {
    actualslideAngleA = (m_slideEncoderA.getPosition()-k_slideEncOffsetA)*2*Math.PI;
    //actualslideAngleB = (m_slideEncoderB.getPosition()-k_slideEncOffsetB)*2*Math.PI;

    m_actualIntakeSpeed = intakeMotorA.get();

    final double intakeFeedForward = m_intakeFeedForward.calculate(m_desiredIntakeSpeed);

    

    //Publish Algae Grabber STuff to the Dashboard
    SmartDashboard.putNumber("slideAngleA", actualslideAngleA);
    //SmartDashboard.putNumber("slideAngleB", actualslideAngleB);
    SmartDashboard.putNumber("IntakeSpeed", m_desiredIntakeSpeed);
    SmartDashboard.putNumber("slideAngleOffsetA", slideAngleOffsetA);
    //SmartDashboard.putNumber("slideAngleOffsetB", slideAngleOffsetB);
    //SmartDashboard.putNumber("Desired slide Angle A", desiredslideDistanceA);
    //SmartDashboard.putNumber("Desired slide Angle B", desiredslideDistanceB);
    SmartDashboard.putNumber("slideMotorACurrent", slideMotorA.getOutputCurrent());
    //SmartDashboard.putNumber("slideMotorBCurrent", slideMotorB.getOutputCurrent());
    SmartDashboard.putNumber("slidePID_ErrorA", m_slidePIDControllerA.getPositionError());
    //SmartDashboard.putNumber("slidePID_ErrorB", m_slidePIDControllerB.getPositionError());    

    //intakeMotorA.set(m_intakePID.calculate(m_actualIntakeSpeed, m_desiredIntakeSpeed));
    //intakeMotorB.set(m_intakePID.calculate(m_actualIntakeSpeed, m_desiredIntakeSpeed));
    intakeMotorA.set(intakeFeedForward + MathUtil.clamp(m_intakePID.calculate(m_actualIntakeSpeed, m_desiredIntakeSpeed),
                        -c_maxIntakeSpeed,
                        c_maxIntakeSpeed));


  }

  /*public void setDesiredslideAngleA(double angle) {
    desiredslideDistanceA = angle + slideAngleOffsetA;
  }*/
  
    /*public void setDesiredslideAngleB(double angle) {
    desiredslideDistanceB= angle + slideAngleOffsetB;
  }*/

  public void incIntakeSlideOffset() {
    slideAngleOffsetA = slideAngleOffsetA + c_slideOffset;
  }

  public void decIntakeSlideOffset() {
    slideAngleOffsetA = slideAngleOffsetA - c_slideOffset;
  }


  

    /*public void extend() {
      setDesiredslideAngleA(k_slideDistanceSetpointA[1]);
      //setDesiredslideAngleB(k_slideDistanceSetpointB[1]);
    }

    public void retract() {
      setDesiredslideAngleA(k_slideDistanceSetpointA[0]);
      //setDesiredslideAngleB(k_slideDistanceSetpointB[0]);
    }*/

    public void setIntakeSpeed(double speed) {
      m_desiredIntakeSpeed = speed;
    }

    public void setDesiredMotorASpeed(double speed) {
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
