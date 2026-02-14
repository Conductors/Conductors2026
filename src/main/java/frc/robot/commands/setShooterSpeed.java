package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterSubsystem;

public class setShooterSpeed extends Command {

  
  private shooterSubsystem m_ShooterSubsystem;
  private double m_speedCmd = 0;
  private boolean m_useDistance = false;
  private double m_distanceFromTag;


  /**
   *Creates a new DriveDistance
   * @param speed The desired speed for the shooter Subsystem
   * @param cs The coralSubsystem subsystem to control
   * @param useDist Ignore speed input, calculate the speed based on Distance (boolean)
  */
  public setShooterSpeed(double speed, shooterSubsystem ss) {
    m_speedCmd = speed;
    m_ShooterSubsystem = ss;
    m_useDistance = false;
    addRequirements(m_ShooterSubsystem);
  }

  public setShooterSpeed(shooterSubsystem ss, boolean useDistance, double dist ) {
    m_speedCmd = 0;
    m_ShooterSubsystem = ss;
    m_useDistance = true;
    m_distanceFromTag = dist;
    addRequirements(m_ShooterSubsystem);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command
    if(m_useDistance)
      m_speedCmd = m_ShooterSubsystem.calcSpeed(m_distanceFromTag);

    

  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    m_ShooterSubsystem.setDesiredMotorASpeed(m_speedCmd);
    m_ShooterSubsystem.setDesiredMotorBSpeed(m_speedCmd);
    
  }

  @Override
  public void end(boolean interrupted) {
    //Run once, at the end of the command
    
  }

  @Override
  public boolean isFinished() {
    // Determines when to finish the command, return true always for speed commands
    return m_ShooterSubsystem.getShoterAIsAtGoal();   
  }

}
