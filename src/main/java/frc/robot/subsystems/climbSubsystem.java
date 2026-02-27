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

    private static final int c_climbMotorPort = 40;
    //private static final int c_climbMotorBPort = 41;

    private SparkMax climbMotor;  
    //private SparkMax climbMotorB;  

    private motorDiagnostics climbDiags;
    //private motorDiagnostics climbBDiags;
    
    
    private static final double c_maxClimbSpeed = 1;
    
    private double m_climbMotorSpeed = 1;
    //private double m_ShooterMotorBSpeed = 1;
    //private double m_turnAngle = 0;

    private double m_desiredMotorSpeed = 0;
    //private double m_desiredMotorBSpeed = 0;
    //private double m_desiredAngle = 0;

    private double o_MotorSpeedOffset = 0;
    //private double o_MotorBSpeedOffset = 0;

    private static final double c_speedOffsetIncrement = 5;


    private RelativeEncoder m_climbEncoder;
    //private RelativeEncoder m_ShooterBEncoder;

    private ProfiledPIDController m_climbPID;
     //ProfiledPIDController m_ShooterBPID;
    //private ProfiledPIDController m_turnMotorPID;
        private static final double Kp_climb = 1;
        private static final double Ki_climb = 0;
        private static final double Kd_climb = 0;
        private static final double Kv_climb = 0;
        private static final double Ks_climb = 0;
        //private static final double Kp_shooterB = 1;
        //private static final double Ki_shooterB = 0;
        //private static final double Kd_shooterB = 0;
        //private static final double Kv_shooterB = 0;
        //private static final double Ks_shooterB = 0;
        //private static final double Kp_turnMotor = 1;
        //private static final double Ki_turnMotor = 0;
        //private static final double Kd_turnMotor = 0;
        //private static final double Kv_turnMotor = 0;
        //private static final double Ks_turnMotor = 0;

    public climbSubsystem() {
        climbMotor = new SparkMax(c_climbMotorPort, SparkLowLevel.MotorType.kBrushless);
        //climbMotorB = new SparkMax(c_climbMotorBPort, SparkLowLevel.MotorType.kBrushless);

        climbDiags = new motorDiagnostics(climbMotor, "Climb Motor");
        //climbBDiags = new motorDiagnostics(climbMotorB, "Shooter B");


        
        m_climbEncoder = climbMotor.getEncoder();
        //m_ShooterBEncoder = climbMotorB.getEncoder();

        m_climbPID =  new ProfiledPIDController(
          Kp_climb,
          Ki_climb,
          Kd_climb,
          new TrapezoidProfile.Constraints(
            Ks_climb,
            Kv_climb));
        m_climbPID.setTolerance(1);

        /*m_ShooterBPID =  new ProfiledPIDController(
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
        m_turnMotorPID.setTolerance(.05);*/
        
    }

    @Override
    public void periodic() {
        m_climbMotorSpeed   = m_climbEncoder.getVelocity();
        //m_ShooterMotorBSpeed   = m_ShooterBEncoder.getVelocity();

        climbMotor.set(MathUtil.clamp(m_climbPID.calculate(m_climbMotorSpeed, m_desiredMotorSpeed),
                                        -c_maxClimbSpeed,
                                        c_maxClimbSpeed));    //need to check motor direction
        //climbMotorB.set(MathUtil.clamp(m_ShooterBPID.calculate(m_desiredMotorBSpeed, m_desiredMotorBSpeed),
        //                                -c_maxShooterBSpeed,
        //                                c_maxShooterBSpeed));
                                        
       
        //Publish Stuff to Dashboard
        SmartDashboard.putNumber("Climb Motor Speed", m_climbMotorSpeed);
        //SmartDashboard.putNumber("Shooter B Speed", m_ShooterMotorBSpeed);

        //SmartDashboard.putNumber("Desired Turn Angle", m_desiredAngle);
        //SmartDashboard.putNumber("Actual Turn Angle", m_turnAngle);

        climbDiags.publishMotorData();
        //climbBDiags.publishMotorData();
        
    }

    public void climb() {
            //TBD...
    }

    public void retract() {
            //TBD...
    }

    public void setDesiredClimbMotorSpeed(double speed) {
        m_desiredMotorSpeed = MathUtil.clamp(speed, -c_maxClimbSpeed, c_maxClimbSpeed);
    }

    public double getDesiredMotorSpeed() {
        return m_desiredMotorSpeed;
    }

    //public void setDesiredMotorBSpeed(double speed) {
    //    m_desiredMotorBSpeed = MathUtil.clamp(speed, -c_ShooterMaxSpeed, c_ShooterMaxSpeed);
    //}

    //public double getDesiredMotorBSpeed() {
    //    return m_desiredMotorBSpeed;
    //}


    // Public functions to all D-Pad to adjust the offset of the Elevator Height
    public void incClimbSpeedOffset() {
        o_MotorSpeedOffset = o_MotorSpeedOffset + c_speedOffsetIncrement;
    }

    public void decShooterASpeedOffset() {
        o_MotorSpeedOffset = o_MotorSpeedOffset - c_speedOffsetIncrement;
    }

    public boolean getClimbIsAtGoal () {
    return m_climbPID.atGoal();
}

        // Public functions to all D-Pad to adjust the offset of the Elevator Height
    /*public void incShooterBSpeedOffset() {
        o_MotorBSpeedOffset = o_MotorBSpeedOffset + c_speedOffsetIncrement;
    }

    public void decShooterBSpeedOffset() {
        o_MotorBSpeedOffset = o_MotorBSpeedOffset - c_speedOffsetIncrement;
    }*/

    
    public void initDefaultCommand() {
        // When Idle, set the speeds to zero            
        Command initSequence = Commands.sequence(
            new InstantCommand(() -> setDesiredClimbMotorSpeed(3)));
            //new InstantCommand(() -> setDesiredMotorBSpeed(4)));
        
        initSequence.addRequirements(this);

        setDefaultCommand(initSequence);
            
    }
    
}

