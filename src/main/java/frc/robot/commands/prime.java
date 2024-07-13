package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;


public class prime extends Command{
    Shooter mShooter;

    public prime(Shooter shooter){
      mShooter = shooter;
      setName("prime");
      addRequirements(shooter);
    }

    @Override
    public void initialize() {
      mShooter.RPMtarget(ShooterConstants.shooterSpeed, ShooterConstants.shooterSpeed);
    }

    @Override 
    public void execute(){
      mShooter.RPMtarget(ShooterConstants.shooterSpeed,ShooterConstants.shooterSpeed);
      if (mShooter.reachedTargetRPM(3500)){
        RobotContainer.controllers.mControls.getHID().setRumble(RumbleType.kBothRumble, 0.3);;
      }
      else{
        RobotContainer.controllers.mControls.getHID().setRumble(RumbleType.kBothRumble, 0);
      }
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
