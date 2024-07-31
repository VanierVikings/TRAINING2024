package frc.robot.commands;

import frc.robot.subsystems.Shooter;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;


public class Prime extends Command{
    Shooter mShooter;

    public Prime(Shooter shooter){
      mShooter = shooter;
      setName("prime");
      addRequirements(shooter);
    }

    @Override
    public void initialize() {
      mShooter.RPMtarget(ShooterConstants.shooterSpeed, ShooterConstants.shooterSpeed);
      if (mShooter.reachedTargetRPM(ShooterConstants.shooterSpeed)){
          RobotContainer.controllers.mControls.getHID().setRumble(RumbleType.kBothRumble, 0.3);;
      }
      else{
        RobotContainer.controllers.mControls.getHID().setRumble(RumbleType.kBothRumble, 0);
      }
    }

    @Override 
    public void execute(){
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
