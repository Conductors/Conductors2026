package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;

public class extendIntake extends Command {
    private intake m_intakeSubsystem;
    //private boolean m_commandExtend = true;
    private boolean extend = false;
    private double m_speedCmd = 0;

    

  /**
   *Creates a new Extend Intake command
   * @param is The intake subsystem to control
   * @param reverse Indicates whether to extend (false) or retract (true)
  */
  /*public slideIntake(boolean extend, intake is) {
    m_intakeSubsystem = is;
    m_commandExtend = extend;
    addRequirements(m_intakeSubsystem);
  }*/

  public extendIntake(boolean extended, intake is) {
    m_intakeSubsystem = is;
    extend = extended;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command
    if (extend = true) {
        m_speedCmd = 0.5;
    } else {
        m_speedCmd = 0;
    }

  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    /*if(m_commandExtend) { 
      m_intakeSubsystem.extend(); 
    } else {
        m_intakeSubsystem.retract();
    }*/
    m_intakeSubsystem.setDesiredMotorASpeed(m_speedCmd);
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
