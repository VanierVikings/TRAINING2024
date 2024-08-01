package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.Command;


public class Amp extends Command{
    private static Shooter m_shooter;
    private static Intake m_intake;

    public Amp(Shooter m_shooter, Intake m_intake){
      Amp.m_shooter = m_shooter;
      Amp.m_intake = m_intake;
      setName("Amp");
      addRequirements(m_shooter, m_intake);
    }

    @Override
    public void initialize() {
      m_shooter.set(ShooterConstants.ampSpeed);
      m_intake.setFeeder(ShooterConstants.primeSpeed);
    }

    @Override
    public void execute() {}
  
    @Override
    public void end(boolean interrupted) {
      m_shooter.stop();
      m_intake.setFeeder(0);
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
