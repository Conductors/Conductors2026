package frc.robot.commands;

import frc.robot.subsystems.intake;
import edu.wpi.first.wpilibj2.command.Command;

public class retractAndIntake extends Command{
    private intake m_intakeSubsystem;
    //private boolean m_commandExtend = true;
    private boolean retract = false;
    private double m_slideSpeedCmd = 0;
    private double m_desiredRetractSpeed;
    private double m_intakeSpeedCmd = 0;
    private boolean m_isReverse = false;

    

  /**
   *Creates a new Retract Intake command
   * @param is The intake subsystem to control
   * @param retracted Indicates whether to extend (false) or retract (true)
  */
  public retractAndIntake(boolean retracted, boolean reverse, intake is, double slideSpeed, double intakeSpeed) {
    m_intakeSubsystem = is;
    retract = retracted;
    m_desiredRetractSpeed = slideSpeed;
    m_isReverse = reverse;
    m_intakeSpeedCmd = m_isReverse? -intakeSpeed:intakeSpeed;   //negate speed if m_isReverse is true 
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command
    if (retract == true) {
        m_slideSpeedCmd = -m_desiredRetractSpeed;
    } else {
        m_slideSpeedCmd = 0;
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
    m_intakeSubsystem.setDesiredSlideSpeed(m_slideSpeedCmd);
    m_intakeSubsystem.setIntakeSpeed(m_intakeSpeedCmd);
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
