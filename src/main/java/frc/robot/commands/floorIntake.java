package frc.robot.commands;

import frc.robot.subsystems.Intake;
import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.Command;

public class FloorIntake extends Command {
  private static Intake m_intake;
  private static int direction;

  public FloorIntake(Intake m_intake, int direction) {
    FloorIntake.m_intake = m_intake;
    FloorIntake.direction = direction;
    setName("Floor Intake");
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.drive(direction * IntakeConstants.speed);
    m_intake.intakeFeed(direction * IntakeConstants.speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
