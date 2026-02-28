package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.intake;
import edu.wpi.first.wpilibj2.command.Command;

public class retractIntake extends Command{
    private intake m_intakeSubsystem;
    //private boolean m_commandExtend = true;
    private boolean retract = false;
    private double m_speedCmd = 0;

    

  /**
   *Creates a new Retract Intake command
   * @param is The intake subsystem to control
   * @param retracted Indicates whether to extend (false) or retract (true)
  */
  public retractIntake(boolean retracted, intake is) {
    m_intakeSubsystem = is;
    retract = retracted;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command
    if (retract == true) {
        m_speedCmd = -Constants.kSlideSpeed;
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
    m_intakeSubsystem.setDesiredSlideSpeed(m_speedCmd);
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
