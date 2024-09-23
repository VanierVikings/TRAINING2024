package frc.robot.commands;

import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
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
    }

    @Override 
    public void execute(){
      m_shooter.set(ShooterConstants.maxRPM);
    }

    @Override
    public void end(boolean interrupted) {
      m_shooter.stop();
      RobotContainer.controllers.mControls.getHID().setRumble(RumbleType.kBothRumble, 0);

    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
