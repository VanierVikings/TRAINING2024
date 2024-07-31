package frc.robot.commands;

import frc.robot.subsystems.Hang;

import edu.wpi.first.wpilibj2.command.Command;


public class HangRetract extends Command{
  Hang mHang;
  double hangSpeed;
  
    public HangRetract(Hang hang, double speed){
        mHang = hang;
        hangSpeed = speed;
        setName("hangRetract");
        addRequirements(hang);
    }

    @Override
    public void initialize() {
      mHang.drive(hangSpeed);
    }
    
    @Override
    public void end(boolean interrupted) {
      mHang.drive(0);
    }
  
    @Override
    public boolean isFinished() {
      return false;
    }
}
