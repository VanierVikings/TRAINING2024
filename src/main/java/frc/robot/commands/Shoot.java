package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;


public class Shoot extends Command{
    Shooter mShooter;
    Intake mIntake;

    public Shoot(Shooter shooter, Intake intake){
      mShooter = shooter;
      mIntake = intake;
      setName("pointAndShoot");
      addRequirements(shooter);
    }

    @Override
    public void initialize() {
      if (mShooter.reachedTargetRPM(ShooterConstants.shooterSpeed)){
        mIntake.intakeFeed(ShooterConstants.feedSpeed);
        mShooter.setShooterFeed(ShooterConstants.feedSpeed);
        return;
      }
    }

    @Override
    public void execute() {

    }
  
    @Override
    public void end(boolean interrupted) {
      mIntake.intakeFeed(0);
      mShooter.setShooterFeed(0);
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
