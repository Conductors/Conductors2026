package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climbSubsystem;

public class setClimbSpeed extends Command {

    private climbSubsystem m_ClimbSubsystem;
    private double m_speedCmd = 0;


  /**
   *Creates a new DriveDistance
   * @param speed The desired speed for the shooter Subsystem
   * @param cs The coralSubsystem subsystem to control
   * @param useDist Ignore speed input, calculate the speed based on Distance (boolean)
  */
  public setClimbSpeed(double speed, climbSubsystem cs) {
    m_speedCmd = speed;
    m_ClimbSubsystem = cs;
    addRequirements(m_ClimbSubsystem);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command

  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    m_ClimbSubsystem.setDesiredClimbMotorSpeed(m_speedCmd);
    
  }

  @Override
  public void end(boolean interrupted) {
    //Run once, at the end of the command
    
  }

  @Override
  public boolean isFinished() {
    // Determines when to finish the command, return true always for speed commands
    return m_ClimbSubsystem.getClimbIsAtGoal();
  }

}

