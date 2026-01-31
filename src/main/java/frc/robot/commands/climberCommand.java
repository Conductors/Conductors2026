package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climbSubsystem;
import frc.robot.subsystems.intake;

public class climberCommand extends Command {

    public enum climbLevel {
      e_floor,
      e_levelOne,
      e_levelTwo
    }

    private climbSubsystem m_climbSubsystem;
    private climbLevel m_climbCommand = climbLevel.e_floor;
    

  /**
   *Creates a new Extend Intake command
   * @param is The intake subsystem to control
   * @param reverse Indicates whether to extend (false) or retract (true)
  */
  public climberCommand(climbLevel lvl, climbSubsystem cs) {
    m_climbSubsystem = cs;
    m_climbCommand = lvl;
    addRequirements(m_climbSubsystem);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command

  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    switch(m_climbCommand)
    {
      case e_floor:
        break;
      case e_levelOne:
        break;
      case e_levelTwo:
        break;
      default:
        break;
    }

      //tbd - need to fill this out
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
