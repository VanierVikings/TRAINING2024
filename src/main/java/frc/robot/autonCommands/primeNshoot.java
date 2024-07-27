package frc.robot.autonCommands;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj2.command.Command;


public class primeNshoot extends Command{
    Shooter mShooter;
    Intake mIntake;

    public primeNshoot(Shooter shooter, Intake intake){
      mShooter = shooter;
      mIntake = intake;
      setName("primeNshoot");
    }

    @Override
    public void initialize() {
      mShooter.RPMtarget(-ShooterConstants.shooterSpeed, -ShooterConstants.shooterSpeed);
    }

    @Override 
    public void execute(){
    if (mShooter.reachedTargetRPM(ShooterConstants.shooterSpeed)){
        mIntake.intakeFeed(ShooterConstants.feedSpeed);
        mShooter.setShooterFeed(ShooterConstants.feedSpeed);
        return;
    }
    }

    @Override
    public void end(boolean interrupted) {
      mShooter.stop();
      mIntake.intakeFeed(0);
      mShooter.setShooterFeed(0);
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
