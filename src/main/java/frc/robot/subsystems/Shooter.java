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
    private final CANSparkMax flywheelRight = new CANSparkMax(ShooterConstants.flywheelRightId,
            MotorType.kBrushless);
    private final CANSparkMax flywheelLeft = new CANSparkMax(ShooterConstants.flywheelLeftId,
            MotorType.kBrushless);
    private final VictorSPX feedMotor = new VictorSPX(ShooterConstants.feedMotorId);

    private RelativeEncoder encoderLeft = flywheelLeft.getEncoder();
    private RelativeEncoder encoderRight = flywheelRight.getEncoder();
    public Shooter() {
        flywheelLeft.setSmartCurrentLimit(ShooterConstants.currentLimit);
        flywheelRight.setSmartCurrentLimit(ShooterConstants.currentLimit);
        flywheelLeft.setIdleMode(IdleMode.kBrake);
        flywheelRight.setIdleMode(IdleMode.kBrake);
    }

    public void setShooterFeed(double speed) {  
        feedMotor.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public void set(double speed) {
        flywheelLeft.set(speed);
        flywheelRight.set(-speed);
    }   

    public void stop() {
        feedMotor.set(VictorSPXControlMode.PercentOutput, 0);
        flywheelRight.stopMotor();
        flywheelLeft.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("LEFT Shooter Encoder", encoderLeft.getVelocity());
        SmartDashboard.putNumber("RIGHT Shooter Encoder", encoderRight.getVelocity());
    }
}

