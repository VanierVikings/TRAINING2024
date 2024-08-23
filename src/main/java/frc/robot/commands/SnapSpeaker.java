package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;


public class SnapSpeaker extends Command{
  private static Drivetrain m_drivetrain;
  private static double angle;

    public SnapSpeaker(Drivetrain m_drivetrain){
      SnapSpeaker.m_drivetrain = m_drivetrain;
      setName("Snap Speaker");
      addRequirements(m_drivetrain);
    }

    @Override
    public void initialize() {
      Pose2d pose = m_drivetrain.nearestAutoAlign(1);
      angle = m_drivetrain.alignmentAngle(pose, m_drivetrain.getPose());
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
