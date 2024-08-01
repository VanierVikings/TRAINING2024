package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;


public class Shoot extends Command{
    private static Shooter m_shooter;
    private static Intake m_intake;

    public Shoot(Shooter m_shooter, Intake m_intake){
      Shoot.m_shooter = m_shooter;
      Shoot.m_intake = m_intake;
      setName("Shoot");
      addRequirements(m_shooter, m_intake);
    }

    @Override
    public void initialize() {
      m_shooter.set(ShooterConstants.maxRPM);
      m_intake.setFeeder(IntakeConstants.speed);
    }

    @Override
    public void execute() {
    }
  
    @Override
    public void end(boolean interrupted) {
      m_shooter.stop();
      m_intake.stopFeeder();
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
