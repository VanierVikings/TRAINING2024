package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;


public class SnapSpeaker extends Command{
  private static Drivetrain m_drivetrain;
  private static double angle;

    public SnapSpeaker(Drivetrain m_drivetrain){
      SnapSpeaker.m_drivetrain = m_drivetrain;
      setName("Snap Speed");
      addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
      new Rotate(m_drivetrain, angle).schedule();
    }
    
    @Override
    public void execute() {
    }
  
    @Override 
    public boolean isFinished() {
      return m_drivetrain.m_roationalPIDController.atSetpoint();
    }
}
