package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private final CANSparkMax shooterPrimeRight = new CANSparkMax(ShooterConstants.shooterPrimeRightId,
            MotorType.kBrushless);
    private final CANSparkMax shooterPrimeLeft = new CANSparkMax(ShooterConstants.shooterPrimeLeftId,
            MotorType.kBrushless);
    private final VictorSPX shooterTopFeed = new VictorSPX(ShooterConstants.shooterTopFeedId);

    private RelativeEncoder encoderLeft = shooterPrimeLeft.getEncoder();
    private RelativeEncoder encoderRight = shooterPrimeRight.getEncoder();
    public Shooter() {
        shooterPrimeLeft.setSmartCurrentLimit(ShooterConstants.currentLimit);
        shooterPrimeRight.setSmartCurrentLimit(ShooterConstants.currentLimit);
        shooterPrimeLeft.setIdleMode(IdleMode.kBrake);
        shooterPrimeRight.setIdleMode(IdleMode.kBrake);
    }

    public void setShooterFeed(double speed) {  
        shooterTopFeed.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public void set(double setpoint) {
        shooterPrimeLeft.set(setpoint);
        shooterPrimeRight.set(-setpoint);
    }   

    public void stop() {
        shooterTopFeed.set(VictorSPXControlMode.PercentOutput, 0);
        shooterPrimeRight.stopMotor();
        shooterPrimeLeft.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LEFT Shooter Encoder", encoderLeft.getVelocity());
        SmartDashboard.putNumber("RIGHT Shooter Encoder", encoderRight.getVelocity());
    }
}

