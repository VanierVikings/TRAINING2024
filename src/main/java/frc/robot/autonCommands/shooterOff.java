package frc.robot.autonCommands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.wpilibj2.command.Command;


public class shooterOff extends Command{
  Shooter mShooter;
  
    public shooterOff(Shooter shooter){
        mShooter = shooter;
        setName("stopShooter");
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
      mShooter.stop();
    }
    
    @Override
    public void end(boolean interrupted) {
      mShooter.stop();
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
