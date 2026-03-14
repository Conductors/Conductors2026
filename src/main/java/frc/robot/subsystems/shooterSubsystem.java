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
import frc.motorDiagnostics;

public class shooterSubsystem extends SubsystemBase {

    private SparkMax shooterMotorA;  
    private SparkMax shooterMotorB;
    private SparkMax conveyorMotor;
    private SparkMax windmillMotor;

    private motorDiagnostics shooterADiags;
    private motorDiagnostics shooterBDiags;
    private motorDiagnostics conveyorDiags;
    private motorDiagnostics windmillDiags;
    
    private int shooterMotorAPort   = 25;
    private int shooterMotorBPort   = 24;
    private int conveyorMotorPort  = 30; 
    private int windmillMotorPort  = 35;

    private static final double c_ShooterMaxSpeed = 5000;

    private double m_ShooterMotorASpeed = 0;
    private double m_ShooterMotorBSpeed = 0;
    private double m_ConveyorMotorSpeed = 0;
    private double m_WindmillMotorSpeed = 0;
    
    private double m_desiredMotorASpeed = 0;
    private double m_desiredMotorBSpeed = 0;
    private double m_desiredConveyorSpeed = 0;
    private double m_desiredWindmillSpeed = 0;
    
    private double o_MotorASpeedOffset = 0;
    
    private static final double c_maxShooterACmd  = 1;
    private static final double c_shooterBSpeed  = .7;
    private static final double c_conveyorSpeed  = .5;
    private static final double c_windmillSpeed  = .5;

    private static final double c_speedOffsetIncrement = 50;

    private RelativeEncoder m_ShooterAEncoder;
    private RelativeEncoder m_ShooterBEncoder;
    private RelativeEncoder m_ConveyorEncoder;
    private RelativeEncoder m_WindmillEncoder;

    private ProfiledPIDController m_ShooterAPID;
    
    private final SimpleMotorFeedforward m_shooterFeedForward = new SimpleMotorFeedforward(0.5,0);
    private static final double Kp_shooterA = .00025;
    private static final double Ki_shooterA = 0;
    private static final double Kd_shooterA = 0;
    private static final double Kv_shooterA = 36000;
    private static final double Ks_shooterA = 6000;

    public shooterSubsystem() {
        shooterMotorA = new SparkMax(shooterMotorAPort, SparkLowLevel.MotorType.kBrushless);
        shooterMotorB = new SparkMax(shooterMotorBPort, SparkLowLevel.MotorType.kBrushless);
        conveyorMotor = new SparkMax(conveyorMotorPort, SparkLowLevel.MotorType.kBrushless);
        windmillMotor = new SparkMax(windmillMotorPort, SparkLowLevel.MotorType.kBrushless);

        shooterADiags = new motorDiagnostics(shooterMotorA, "Shooter A");
        shooterBDiags = new motorDiagnostics(shooterMotorB, "Shooter B");
        conveyorDiags = new motorDiagnostics(conveyorMotor, "Conveyor");
        windmillDiags = new motorDiagnostics(windmillMotor, "WindMill");


        //m_turnEncoder = new DutyCycleEncoder(turnEncoderPort);
        m_ShooterAEncoder = shooterMotorA.getEncoder();
        m_ShooterBEncoder = shooterMotorB.getEncoder();
        m_ConveyorEncoder = conveyorMotor.getEncoder();
        m_WindmillEncoder = windmillMotor.getEncoder();

        m_ShooterAPID =  new ProfiledPIDController(
          Kp_shooterA,
          Ki_shooterA,
          Kd_shooterA,
          new TrapezoidProfile.Constraints(
            Ks_shooterA,
            Kv_shooterA));
        m_ShooterAPID.setTolerance(1);
        
    }

    @Override
    public void periodic() {
        m_ShooterMotorASpeed   = m_ShooterAEncoder.getVelocity();
        m_ShooterMotorBSpeed   = m_ShooterBEncoder.getVelocity();
        //m_ConveyorMotorSpeed   = m_ConveyorEncoder.getVelocity();
        m_WindmillMotorSpeed   = m_WindmillEncoder.getVelocity();

        final double shooterFeedForward = m_shooterFeedForward.calculate(m_desiredMotorASpeed);

        shooterMotorA.set(shooterFeedForward + MathUtil.clamp(m_ShooterAPID.calculate(m_ShooterMotorASpeed, m_desiredMotorASpeed),
                                        -c_maxShooterACmd,
                                        c_maxShooterACmd));    //need to check motor direction
        
        shooterMotorB.set(m_desiredMotorBSpeed);
        //conveyorMotor.set(m_desiredConveyorSpeed);
        windmillMotor.set(m_desiredWindmillSpeed);
            

        //Publish Stuff to Dashboard
        SmartDashboard.putNumber("Desired ShooterA Speed", m_desiredMotorASpeed);
        SmartDashboard.putNumber("Desired ShooterB Speed", m_desiredMotorBSpeed);
        SmartDashboard.putNumber("Desired Conveyor Speed", m_desiredConveyorSpeed);
        SmartDashboard.putNumber("Desired Windmill Speed", m_desiredWindmillSpeed);
        SmartDashboard.putNumber("Shooter A Speed", m_ShooterMotorASpeed);
        SmartDashboard.putNumber("Shooter B Speed", m_ShooterMotorBSpeed);
        SmartDashboard.putNumber("Conveyor Speed", m_ConveyorMotorSpeed);
        SmartDashboard.putNumber("Windmill Speed", m_WindmillMotorSpeed);
        SmartDashboard.putNumber("Shooter A Offset", o_MotorASpeedOffset);
                

        shooterADiags.publishMotorData();
        shooterBDiags.publishMotorData();
        conveyorDiags.publishMotorData();
        windmillDiags.publishMotorData();
        
    }

    public void setDesiredMotorASpeed(double speed) {
        if(speed == 0)
        {
            m_desiredMotorASpeed = 0;
        } else {
            m_desiredMotorASpeed = MathUtil.clamp(speed + o_MotorASpeedOffset, -c_ShooterMaxSpeed, c_ShooterMaxSpeed);
        }
    }

    public double getDesiredMotorASpeed() {
        return m_desiredMotorASpeed;
    }

    public void setDesiredMotorBSpeed(double speed) {
        if(speed == 0) {
            m_desiredMotorBSpeed = 0;
            m_desiredConveyorSpeed = 0;
            m_desiredWindmillSpeed = 0;
        } else if(speed > 0) {
            m_desiredMotorBSpeed = -c_shooterBSpeed;
            m_desiredConveyorSpeed = -c_conveyorSpeed;
            m_desiredWindmillSpeed = -c_windmillSpeed;
        } else if(speed < 0) {
            m_desiredMotorBSpeed = c_shooterBSpeed;
            m_desiredConveyorSpeed = c_conveyorSpeed;
            m_desiredWindmillSpeed = c_windmillSpeed;
        }
    }

    public double getDesiredMotorBSpeed() {
        return m_desiredMotorBSpeed;
    }

    public double getDesiredConveyorSpeed() {
        return m_desiredConveyorSpeed;
    }

    public double getDesiredWindmillSpeed() {
        return m_desiredWindmillSpeed;
    }

    public double getMotorASpeed() {
        return m_ShooterMotorASpeed;
    }

    public double getMotorBSpeed() {
        return m_ShooterMotorBSpeed;
    }

    public double getConveyorSpeed() {
        return m_ConveyorMotorSpeed;
    }

    public double getWindmillSpeed() {
        return m_WindmillMotorSpeed;
    }

    // Public functions to all D-Pad to adjust the offset of the Shooter (A) Speed
    public void incShooterASpeedOffset() {
        o_MotorASpeedOffset = o_MotorASpeedOffset + c_speedOffsetIncrement;
    }

    public void decShooterASpeedOffset() {
        o_MotorASpeedOffset = o_MotorASpeedOffset - c_speedOffsetIncrement;
    }

    public boolean getShoterAIsAtGoal () {
        return m_ShooterAPID.atGoal();
    }

    /**
     * 
     * @param dist The distance from the camera, in meters
     * @return The Shooter A Motor speeds (in rpm) appropriate for a given distance
     */
    public double calcSpeed(double dist) {
        
        //setSpeed = dist;      //Change this for different distances (calibration logic only)
        double setSpeed = -56.93*dist*dist + 1140.05*dist + 1116.609;

        return setSpeed;
    }
    
    public void initDefaultCommand() {
        // When Idle, set the speeds to zero            
        Command initSequence = Commands.sequence(
            new InstantCommand(() -> setDesiredMotorASpeed(0)),
            new InstantCommand(() -> setDesiredMotorBSpeed(0)));
        
        initSequence.addRequirements(this);

        setDefaultCommand(initSequence);
            
    }
    
}

