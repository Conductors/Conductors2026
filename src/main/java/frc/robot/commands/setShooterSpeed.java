package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.shooterSubsystem;

public class setShooterSpeed extends Command {

  
  private shooterSubsystem m_ShooterSubsystem;
  private double m_speedCmd = 0;
  private boolean m_useDistance = false;
  private double m_distanceFromTag;
  private Timer m_delayTimer;
  private double c_MotorBDelay = .5;
  private Robot robotRef;


  /**
   *Creates a new DriveDistance
   * @param speed The desired speed for the shooter Subsystem
   * @param cs The coralSubsystem subsystem to control
   * @param useDist Ignore speed input, calculate the speed based on Distance (boolean)
  */
  public setShooterSpeed(double speed, shooterSubsystem ss, Robot robot, boolean isShooterRunning) {
    m_speedCmd = speed;
    m_ShooterSubsystem = ss;
    System.out.println("running");
    // if(speed > 0){
    //  robot.shooterRunning = true;
    // }else{
    //   robot.shooterRunning = false;
    // }
    robot.shooterRunning = isShooterRunning;
    m_useDistance = false;
    m_delayTimer = new Timer();
    addRequirements(m_ShooterSubsystem);
  }

  public setShooterSpeed(shooterSubsystem ss, boolean useDistance, Robot robot, boolean isShooterRunning) {
    m_speedCmd = 0;
    m_ShooterSubsystem = ss;
    m_useDistance = true;
    robotRef = robot;
    robot.shooterRunning = isShooterRunning;
    m_delayTimer = new Timer();
    addRequirements(m_ShooterSubsystem);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command
    if(m_useDistance){
      // double distanceFromTag = robotRef.getDistanceFromHubCenter();
      double distanceFromTag = robotRef.GetDistanceFromHubWithPosition();
      System.out.println("distFromTag="+distanceFromTag);
    
     if (distanceFromTag == 0){
      m_distanceFromTag = 2.2;
    }else{
      m_distanceFromTag = distanceFromTag;
    }
    System.out.println(distanceFromTag);
    m_speedCmd = -m_ShooterSubsystem.calcSpeed(m_distanceFromTag);
  }

    m_delayTimer.reset();
    m_delayTimer.start();

  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    m_ShooterSubsystem.setDesiredMotorASpeed(-m_speedCmd);
    
    
  }

  @Override
  public void end(boolean interrupted) {
    //Run once, at the end of the command

    /* Steve changed this 2/28 - theory is that ShooterA will PID up to set speed, 
    and when it's at goal, this END function will trigger B motor to start.  Moved
    the motor B speed command to the end() function (previously in  execute() )
    */
    m_ShooterSubsystem.setDesiredMotorBSpeed(-m_speedCmd);   
    System.out.println("end Motor B commanded");

    m_delayTimer.stop();
    m_delayTimer.reset();

  }

  @Override
  public boolean isFinished() {
    // Determines when to finish the command, return true always for speed commands
    boolean rv = (m_delayTimer.get() > c_MotorBDelay);
    return rv;
  }

}
