package frc.robot.commands;

import frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class FloorIntake extends Command {
  private static Intake m_intake;
  private static double speed;

  public FloorIntake(Intake m_intake, double speed) {
    FloorIntake.m_intake = m_intake;
    FloorIntake.speed = speed;
    setName("Floor Intake");
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
    m_intake.setIntake(speed);
    m_intake.setFeeder(speed);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.stopIntake();
    m_intake.stopFeeder();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
