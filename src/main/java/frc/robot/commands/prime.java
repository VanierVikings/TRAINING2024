package frc.robot.commands;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;


public class Prime extends Command{
    private static Shooter m_shooter;

    public Prime(Shooter m_shooter){
      Prime.m_shooter = m_shooter;
      setName("Prime");
      addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
      m_shooter.set(ShooterConstants.maxRPM);
    }

    @Override 
    public void execute(){}

    @Override
    public void end(boolean interrupted) {
      m_shooter.stop();
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
