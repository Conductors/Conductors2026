package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooterSubsystem extends SubsystemBase {



    private SparkMax shooterMotorA;  
    private SparkMax shooterMotorB;
    private SparkMax turnMotor;
    
    private int shooterMotorAPort   = 24;
    private int shooterMotorBPort   = 25;
    private int turnMotorPort       = 26;

    private DutyCycleEncoder m_turnEncoder;
    private int turnEncoderPort = 5;
    //private static final double c_TurnEncoderOffset = 0;  //note that we can just use the raw angle?

    private static final double c_ShooterMaxSpeed = 1;
    private static final double c_TurnMinAngle = 0;
    private static final double c_TurnMaxAngle = 0;
    
    private double m_ShooterMotorASpeed = 0;
    private double m_ShooterMotorBSpeed = 0;
    private double m_turnAngle = 0;

    private double m_desiredMotorASpeed = 0;
    private double m_desiredMotorBSpeed = 0;
    private double m_desiredAngle = 0;

    private double o_MotorASpeedOffset = 0;
    private double o_MotorBSpeedOffset = 0;

    private static final double c_maxShooterASpeed  = 1000;
    private static final double c_maxShooterBSpeed  = 1000;
    private static final double c_maxTurnSpeed      = 1000;

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

    public shooterSubsystem() {
        shooterMotorA = new SparkMax(shooterMotorAPort, SparkLowLevel.MotorType.kBrushless);
        shooterMotorB = new SparkMax(shooterMotorBPort, SparkLowLevel.MotorType.kBrushless);

        turnMotor = new SparkMax(turnMotorPort, SparkLowLevel.MotorType.kBrushless);
        SparkMaxConfig tiltConfig = new SparkMaxConfig();
            tiltConfig.idleMode(IdleMode.kBrake);
        turnMotor.configure(tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_turnEncoder = new DutyCycleEncoder(turnEncoderPort);
        m_ShooterAEncoder = shooterMotorA.getEncoder();
        m_ShooterBEncoder = shooterMotorB.getEncoder();

        m_ShooterAPID =  new ProfiledPIDController(
          Kp_shooterA,
          Ki_shooterA,
          Kd_shooterA,
          new TrapezoidProfile.Constraints(
            Ks_shooterA,
            Kv_shooterA));
        m_ShooterAPID.setTolerance(.05);

        m_ShooterBPID =  new ProfiledPIDController(
          Kp_shooterB,
          Ki_shooterB,
          Kd_shooterB,
          new TrapezoidProfile.Constraints(
            Ks_shooterB,
            Kv_shooterB));
        m_ShooterBPID.setTolerance(.05);
    
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
      m_turnAngle = m_turnEncoder.get();

      shooterMotorA.set(MathUtil.clamp(m_ShooterAPID.calculate(m_ShooterMotorASpeed, m_desiredMotorASpeed),
                                        -c_maxShooterASpeed,
                                        c_maxShooterASpeed));    //need to check motor direction
      shooterMotorB.set(MathUtil.clamp(m_ShooterBPID.calculate(m_desiredMotorBSpeed, m_desiredMotorBSpeed),
                                        -c_maxShooterBSpeed,
                                        c_maxShooterBSpeed));
                                        
    
      turnMotor.set(-MathUtil.clamp(m_turnMotorPID.calculate(m_turnAngle, m_desiredAngle),
                                        -c_maxTurnSpeed,
                                        c_maxTurnSpeed));
        
        //Publish Stuff to Dashboard
        SmartDashboard.putNumber("Shooter A Speed", m_ShooterMotorASpeed);
        SmartDashboard.putNumber("Shooter B Speed", m_ShooterMotorBSpeed);
        SmartDashboard.putNumber("Turn Motor Speed", turnMotor.get());

        SmartDashboard.putNumber("Desired Turn Angle", m_desiredAngle);
        SmartDashboard.putNumber("Actual Turn Angle", m_turnAngle);
        
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

    /**
     *@param angle Angle in radians
     */
    public void setDesiredTurnAngle(double angle) {
        m_desiredAngle = MathUtil.clamp(angle,c_TurnMinAngle,c_TurnMaxAngle);
    }

    public double getDesiredTurnAngle() {
        return m_desiredAngle;
    }

    public double getActualTiltAngle() {
        return m_turnAngle;
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
        setDefaultCommand(Commands.sequence(
            new InstantCommand(() -> setDesiredMotorASpeed(0)),
            new InstantCommand(() -> setDesiredMotorBSpeed(0))));
    }
    
}

