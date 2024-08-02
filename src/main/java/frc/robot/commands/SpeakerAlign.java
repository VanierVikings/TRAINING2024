package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    Pose2d prealignmnent = m_drivetrain.nearestAutoAlign(m_drivetrain.posesPreAlignment);

    double rotationAngle = m_drivetrain.alignmentAngle(prealignmnent, m_drivetrain.getPose());
    int index = m_drivetrain.posesPreAlignment.indexOf(prealignmnent);

    m_drivetrain.rotate(rotationAngle);
    SmartDashboard.putNumber("To Speaker Angle", rotationAngle);
    SmartDashboard.putNumber("Alignment Index", index);

    PathConstraints constraints = new PathConstraints(3.0, 4.0, 0, 0);
    AutoBuilder.pathfindToPose(prealignmnent, constraints, 0.0, 0.0);

    Pose2d aligned = m_drivetrain.posesAligned.get(index);

    m_drivetrain.rotate(aligned.getRotation().getDegrees());

    constraints = new PathConstraints(0.5, 0.5, 0, 0);
    AutoBuilder.pathfindToPose(aligned, constraints, 0.0, 0.0);
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
