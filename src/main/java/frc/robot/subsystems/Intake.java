package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    
  private SparkMax tiltMotorA;  
  private SparkMax tiltMotorB;
  private SparkMax intakeMotorA;
  private SparkMax intakeMotorB;
  
  private DutyCycleEncoder m_tiltEncoderA;
  private DutyCycleEncoder m_tiltEncoderB;
  
  private double desiredTiltAngleA = Constants.intakeConstants.k_tiltAngleSetpointA[0];
  private double desiredTiltAngleB = Constants.intakeConstants.k_tiltAngleSetpointB[0];
  private double actualTiltAngleA = 0;
  private double actualTiltAngleB = 0;
  private double TiltAngleOffsetA = 0;
  private double TiltAngleOffsetB = 0;
  private double TiltAngleStep = 0.1;
  
  private final ProfiledPIDController m_tiltPIDControllerA;
  private final ProfiledPIDController m_tiltPIDControllerB;

  private double desiredIntakeSpeed = 0;

  /**
   * @param speed The speed at which the robot will drive
   * @param drive The drive subsystem on which this command will run
   */
  public intake(int intakeMotorPortA,
                        int intakeMotorPortB,
                        int tiltMotorPortA,
                        int tiltMotorPortB,
                        int tiltEncoderPortA,
                        int tiltEncoderPortB) {

    intakeMotorA = new SparkMax(intakeMotorPortA, SparkLowLevel.MotorType.kBrushless);
    intakeMotorB = new SparkMax(intakeMotorPortB, SparkLowLevel.MotorType.kBrushless);
    tiltMotorA = new SparkMax(tiltMotorPortA, SparkLowLevel.MotorType.kBrushless);
    tiltMotorB = new SparkMax(tiltMotorPortB, SparkLowLevel.MotorType.kBrushless);
    m_tiltEncoderA = new DutyCycleEncoder(tiltEncoderPortA);
    m_tiltEncoderB = new DutyCycleEncoder(tiltEncoderPortB);
    m_tiltEncoderA.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0);
    m_tiltEncoderB.setDutyCycleRange(1.0/1025.0, 1024.0/1025.0);
    

    m_tiltPIDControllerA =
      new ProfiledPIDController(
          Constants.KpTilt,
          0,
          0,
          new TrapezoidProfile.Constraints(
              Constants.tiltMaxVelocity,
              Constants.tiltMaxAccel));
    m_tiltPIDControllerA.setTolerance(.05);  //sets the tolerance for the PID controller, in meters

    m_tiltPIDControllerB =
      new ProfiledPIDController(
          Constants.KpTilt,
          0,
          0,
          new TrapezoidProfile.Constraints(
              Constants.tiltMaxVelocity,
              Constants.tiltMaxAccel));
    m_tiltPIDControllerB.setTolerance(.05);  //sets the tolerance for the PID controller, in meters
    
    // Set the default command for a subsystem here. (set the claw speed to 0)
    //setDefaultCommand(new setClawSpeed(0, this));
  }

  @Override
  public void periodic() {
    actualTiltAngleA = (m_tiltEncoderA.get()-Constants.intakeConstants.k_tiltEncOffsetA)*2*Math.PI;
    actualTiltAngleB = (m_tiltEncoderB.get()-Constants.intakeConstants.k_tiltEncOffsetB)*2*Math.PI;

    //Publish Algae Grabber STuff to the Dashboard
    SmartDashboard.putNumber("TiltAngleA", actualTiltAngleA);
    SmartDashboard.putNumber("TiltAngleB", actualTiltAngleB);
    SmartDashboard.putNumber("IntakeSpeed", desiredIntakeSpeed);
    SmartDashboard.putNumber("TiltAngleOffsetA", TiltAngleOffsetA);
    SmartDashboard.putNumber("TiltAngleOffsetB", TiltAngleOffsetB);
    SmartDashboard.putNumber("Desired Tilt Angle A", desiredTiltAngleA);
    SmartDashboard.putNumber("Desired Tilt Angle B", desiredTiltAngleB);
    SmartDashboard.putNumber("tiltMotorACurrent", tiltMotorA.getOutputCurrent());
    SmartDashboard.putNumber("tiltMotorBCurrent", tiltMotorB.getOutputCurrent());
    SmartDashboard.putNumber("TiltPID_ErrorA", m_tiltPIDControllerA.getPositionError());
    SmartDashboard.putNumber("TiltPID_ErrorB", m_tiltPIDControllerB.getPositionError());

    

    //craneMotor.set(-m_CranePIDController.calculate(actualCraneAngle, desiredCraneAngle));    //need to check motor direction
    //wristMotor.set(m_WristPIDController.calculate(actualWristAngle, desiredWristAngle));

    //clawMotorUpper.set(desiredClawSpeed);
    //clawMotorLower.set(-desiredClawSpeed);

  }

  public void setDesiredTiltAngleA(double angle) {
    desiredTiltAngleA = angle + TiltAngleOffsetA;
  }
  
    public void setDesiredTiltAngleB(double angle) {
    desiredTiltAngleB= angle + TiltAngleOffsetB;
  }

    
}
