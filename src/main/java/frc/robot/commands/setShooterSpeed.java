package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooterSubsystem;

public class setShooterSpeed extends Command {

  
  private shooterSubsystem m_ShooterSubsystem;
  private double m_speedCmd = 0;

  /**
   *Creates a new DriveDistance
   * @param speed The desired speed for the shooter Subsystem
   * @param cs The coralSubsystem subsystem to control
  */
  public setShooterSpeed(double speed, shooterSubsystem ss) {
    m_speedCmd = speed;
    m_ShooterSubsystem = ss;
    addRequirements(m_ShooterSubsystem);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command

  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    m_ShooterSubsystem.setDesiredMotorASpeed(m_speedCmd);
  }

  @Override
  public void end(boolean interrupted) {
    //Run once, at the end of the command
    
  }

  @Override
  public boolean isFinished() {
    // Determines when to finish the command, return true always for speed commands
    return true;   
  }

}
