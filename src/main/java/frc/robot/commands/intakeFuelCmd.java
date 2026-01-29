package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake;

public class intakeFuelCmd extends Command {

  
  private intake m_intakeSubsystem;
  private double m_speedCmd = 0;
  private boolean m_isReverse = false;

  /**
   *Creates a new Intake Fuel Command
   * @param speed The desired speed for the shooter Subsystem
   * @param is The intake subsystem to control
   * @param reverse Indicates whether to negate the motor speeds (spit out the fuel)
  */
  public intakeFuelCmd(double speed, intake is, boolean reverse) {
    m_intakeSubsystem = is;
    m_isReverse = reverse;
    m_speedCmd = m_isReverse? -speed:speed;   //negate speed if m_isReverse is true    
    addRequirements(m_intakeSubsystem);
  }

  /**
   *Creates a new Intake Fuel Command (defaulted to forward)
   * @param speed The desired speed for the shooter Subsystem
   * @param is The coralSubsystem subsystem to control
  */
  public intakeFuelCmd(double speed, intake ss) {
    m_speedCmd = speed;
    m_intakeSubsystem = ss;
    m_isReverse = false;
    addRequirements(m_intakeSubsystem);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command

  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    m_intakeSubsystem.setIntakeSpeed(m_speedCmd);
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
