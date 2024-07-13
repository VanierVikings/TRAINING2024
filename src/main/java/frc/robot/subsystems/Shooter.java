package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkBase shooterPrimeRight = new CANSparkMax(ShooterConstants.shooterPrimeRightId, MotorType.kBrushless);
    private final CANSparkBase shooterPrimeLeft = new CANSparkMax(ShooterConstants.shooterPrimeLeftId, MotorType.kBrushless);
    private final VictorSPX shooterTopFeed = new VictorSPX(ShooterConstants.shooterTopFeedId);
    private PIDController  leftMotorPID = new PIDController(0,0,0);
    private PIDController  rightMotorPID = new PIDController(0,0,0);
    private double targetRPM = ShooterConstants.shooterSpeed;
    public double currentRPM = 0;

    public Shooter() {
        shooterPrimeLeft.restoreFactoryDefaults();
        shooterPrimeRight.restoreFactoryDefaults();
        shooterPrimeLeft.setOpenLoopRampRate(0);
        shooterPrimeRight.setOpenLoopRampRate(0);

    }

    public void setShooterFeed(double speed){
        shooterPrimeLeft.setIdleMode(IdleMode.kBrake);
        shooterPrimeRight.setIdleMode(IdleMode.kBrake);
        shooterTopFeed.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public double getTargetRPM(){
        return targetRPM;
    }

    public void changeTargetRPM(double targetRPM){
        this.targetRPM = targetRPM;

    }

    public void RPMtarget(double LEFTSPEED, double RIGHTSPEED){
        shooterPrimeLeft.setVoltage(leftMotorPID.calculate(shooterPrimeLeft.getEncoder().getVelocity(), LEFTSPEED));
        shooterPrimeRight.setVoltage(rightMotorPID.calculate(shooterPrimeRight.getEncoder().getVelocity(), RIGHTSPEED));
    }

    public boolean reachedTargetRPM(double target){
        return (Math.abs(currentRPM) >= Math.abs((targetRPM - Constants.DriverConstants.shooterTolerance)));
    }

    public void setShooterPrime(double speed){
        shooterPrimeRight.set(speed);
        shooterPrimeLeft.set(-speed);
    }

    public double leftRPM(){
        return shooterPrimeLeft.getEncoder().getVelocity();
    }

    public void extend(){}

    public void stop(){
        shooterTopFeed.set(VictorSPXControlMode.PercentOutput, 0);
        shooterPrimeRight.stopMotor();
        shooterPrimeLeft.stopMotor();
    }

    @Override 
    public void periodic(){
        currentRPM = leftRPM();
    }
}
