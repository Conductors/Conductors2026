package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.motorDiagnostics;

public class climbSubsystem extends SubsystemBase {

    private static final int c_climbMotorAPort = 40;
    private static final int c_climbMotorBPort = 41;

    private SparkMax climbMotorA;  
    private SparkMax climbMotorB;  

    private motorDiagnostics climbADiags;
    private motorDiagnostics climbBDiags;
    
    
    private static final double c_ShooterMaxSpeed = 1;
    
    private double m_ShooterMotorASpeed = 1;
    private double m_ShooterMotorBSpeed = 1;
    private double m_turnAngle = 0;

    private double m_desiredMotorASpeed = 0;
    private double m_desiredMotorBSpeed = 0;
    private double m_desiredAngle = 0;

    private double o_MotorASpeedOffset = 0;
    private double o_MotorBSpeedOffset = 0;

    private static final double c_maxShooterASpeed  = 1;
    private static final double c_maxShooterBSpeed  = 1;


    private static final double c_speedOffsetIncrement = 5;


    private RelativeEncoder m_ShooterAEncoder;
    private RelativeEncoder m_ShooterBEncoder;

    private ProfiledPIDController m_ShooterAPID;
    private ProfiledPIDController m_ShooterBPID;
    private ProfiledPIDController m_turnMotorPID;
        private static final double Kp_shooterA = 1;
        private static final double Ki_shooterA = 0;
        private static final double Kd_shooterA = 0;
        private static final double Kv_shooterA = 0;
        private static final double Ks_shooterA = 0;
        private static final double Kp_shooterB = 1;
        private static final double Ki_shooterB = 0;
        private static final double Kd_shooterB = 0;
        private static final double Kv_shooterB = 0;
        private static final double Ks_shooterB = 0;
        private static final double Kp_turnMotor = 1;
        private static final double Ki_turnMotor = 0;
        private static final double Kd_turnMotor = 0;
        private static final double Kv_turnMotor = 0;
        private static final double Ks_turnMotor = 0;

    public climbSubsystem() {
        climbMotorA = new SparkMax(c_climbMotorAPort, SparkLowLevel.MotorType.kBrushless);
        climbMotorB = new SparkMax(c_climbMotorBPort, SparkLowLevel.MotorType.kBrushless);

        climbADiags = new motorDiagnostics(climbMotorA, "Shooter A");
        climbBDiags = new motorDiagnostics(climbMotorB, "Shooter B");


        
        m_ShooterAEncoder = climbMotorA.getEncoder();
        m_ShooterBEncoder = climbMotorB.getEncoder();

        m_ShooterAPID =  new ProfiledPIDController(
          Kp_shooterA,
          Ki_shooterA,
          Kd_shooterA,
          new TrapezoidProfile.Constraints(
            Ks_shooterA,
            Kv_shooterA));
        m_ShooterAPID.setTolerance(1);

        m_ShooterBPID =  new ProfiledPIDController(
          Kp_shooterB,
          Ki_shooterB,
          Kd_shooterB,
          new TrapezoidProfile.Constraints(
            Ks_shooterB,
            Kv_shooterB));
        m_ShooterBPID.setTolerance(1);
    
        m_turnMotorPID =  new ProfiledPIDController(
          Kp_turnMotor,
          Ki_turnMotor,
          Kd_turnMotor,
          new TrapezoidProfile.Constraints(
            Ks_turnMotor,
            Kv_turnMotor));
        m_turnMotorPID.setTolerance(.05);
        
    }

    @Override
    public void periodic() {
        m_ShooterMotorASpeed   = m_ShooterAEncoder.getVelocity();
        m_ShooterMotorBSpeed   = m_ShooterBEncoder.getVelocity();

        climbMotorA.set(MathUtil.clamp(m_ShooterAPID.calculate(m_ShooterMotorASpeed, m_desiredMotorASpeed),
                                        -c_maxShooterASpeed,
                                        c_maxShooterASpeed));    //need to check motor direction
        climbMotorB.set(MathUtil.clamp(m_ShooterBPID.calculate(m_desiredMotorBSpeed, m_desiredMotorBSpeed),
                                        -c_maxShooterBSpeed,
                                        c_maxShooterBSpeed));
                                        
       
        //Publish Stuff to Dashboard
        SmartDashboard.putNumber("Shooter A Speed", m_ShooterMotorASpeed);
        SmartDashboard.putNumber("Shooter B Speed", m_ShooterMotorBSpeed);

        SmartDashboard.putNumber("Desired Turn Angle", m_desiredAngle);
        SmartDashboard.putNumber("Actual Turn Angle", m_turnAngle);

        climbADiags.publishMotorData();
        climbBDiags.publishMotorData();
        
    }

    public void climb() {
            //TBD...
    }

    public void retract() {
            //TBD...
    }

    public void setDesiredMotorASpeed(double speed) {
        m_desiredMotorASpeed = MathUtil.clamp(speed, -c_ShooterMaxSpeed, c_ShooterMaxSpeed);
    }

    public double getDesiredMotorASpeed() {
        return m_desiredMotorASpeed;
    }

    public void setDesiredMotorBSpeed(double speed) {
        m_desiredMotorBSpeed = MathUtil.clamp(speed, -c_ShooterMaxSpeed, c_ShooterMaxSpeed);
    }

    public double getDesiredMotorBSpeed() {
        return m_desiredMotorBSpeed;
    }


    // Public functions to all D-Pad to adjust the offset of the Elevator Height
    public void incShooterASpeedOffset() {
        o_MotorASpeedOffset = o_MotorASpeedOffset + c_speedOffsetIncrement;
    }

    public void decShooterASpeedOffset() {
        o_MotorASpeedOffset = o_MotorASpeedOffset - c_speedOffsetIncrement;
    }

        // Public functions to all D-Pad to adjust the offset of the Elevator Height
    public void incShooterBSpeedOffset() {
        o_MotorBSpeedOffset = o_MotorBSpeedOffset + c_speedOffsetIncrement;
    }

    public void decShooterBSpeedOffset() {
        o_MotorBSpeedOffset = o_MotorBSpeedOffset - c_speedOffsetIncrement;
    }

    
    public void initDefaultCommand() {
        // When Idle, set the speeds to zero            
        Command initSequence = Commands.sequence(
            new InstantCommand(() -> setDesiredMotorASpeed(3)),
            new InstantCommand(() -> setDesiredMotorBSpeed(4)));
        
        initSequence.addRequirements(this);

        setDefaultCommand(initSequence);
            
    }
    
}

