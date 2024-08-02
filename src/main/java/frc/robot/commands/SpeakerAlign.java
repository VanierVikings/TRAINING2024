package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class SpeakerAlign extends Command {
  private static Drivetrain m_drivetrain;

  public SpeakerAlign(Drivetrain m_drivetrain) {
    SpeakerAlign.m_drivetrain = m_drivetrain;
    setName("Speaker Align");
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    Pose2d target = m_drivetrain.nearestAutoAlign();
    m_drivetrain.rotate(target.getRotation().getDegrees());
    PathConstraints constraints = new PathConstraints(3.0, 4.0, 0, 0);
    AutoBuilder.pathfindToPose(target, constraints, 0.0, 0.0);
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