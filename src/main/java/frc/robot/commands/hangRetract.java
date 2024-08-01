package frc.robot.commands;

import frc.robot.subsystems.Hang;

import edu.wpi.first.wpilibj2.command.Command;


public class HangRetract extends Command{
  private static Hang m_hang;
  double speed;
  
    public HangRetract(Hang m_hang, double speed){
        HangRetract.m_hang = m_hang;
        this.speed = speed;
        setName("Hang Retract");
        addRequirements(m_hang);
    }

    @Override
    public void initialize() {
      m_hang.set(speed);
    }
    
    @Override
    public void end(boolean interrupted) {
      m_hang.set(0);
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
