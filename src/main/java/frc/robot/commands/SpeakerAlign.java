package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SpeakerAlign extends Command {
  private static Drivetrain m_drivetrain;
  private static double angle;
  private static Pose2d pose;

  public SpeakerAlign(Drivetrain m_drivetrain) {
    SpeakerAlign.m_drivetrain = m_drivetrain;
    setName("Speaker Align");
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    pose = m_drivetrain.nearestAutoAlign(0);
    angle = m_drivetrain.alignmentAngle(pose, m_drivetrain.getPose());

    new Rotate(m_drivetrain, angle).schedule(); 
    
    PathConstraints constraints = new PathConstraints(3.0, 4.0, 3, 4);
    AutoBuilder.pathfindToPose(pose, constraints, 1, 1).schedule();

    pose = m_drivetrain.nearestAutoAlign(1);
    angle = m_drivetrain.alignmentAngle(pose, m_drivetrain.getPose());

    new Rotate(m_drivetrain, angle).schedule();

    constraints = new  PathConstraints(1, 1, 1, 1);
    AutoBuilder.pathfindToPose(pose, constraints, 0.0, 0.0).schedule();
  }

  @Override
  public void execute() {
  }


  @Override
  public void end(boolean interrupted) {
    m_drivetrain.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}