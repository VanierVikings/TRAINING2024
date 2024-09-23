package frc.robot.commands;

import frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;


public class TopIntake extends Command{
    private static Shooter m_shooter;

    public TopIntake(Shooter m_shooter){
      TopIntake.m_shooter = m_shooter;
      setName("Top Intake");
      addRequirements(m_shooter);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void end(boolean interrupted) {
      m_shooter.stop();
    }

    @Override
    public boolean isFinished() {
      return false;
    }
}
