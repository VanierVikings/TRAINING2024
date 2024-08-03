package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
    private final CANSparkBase shooterPrimeRight = new CANSparkMax(ShooterConstants.shooterPrimeRightId,
            MotorType.kBrushless);
    private final CANSparkBase shooterPrimeLeft = new CANSparkMax(ShooterConstants.shooterPrimeLeftId,
            MotorType.kBrushless);
    private final VictorSPX shooterTopFeed = new VictorSPX(ShooterConstants.shooterTopFeedId);

    private RelativeEncoder encoderLeft = shooterPrimeLeft.getEncoder();
    private RelativeEncoder encoderRight = shooterPrimeRight.getEncoder();

    private final PIDController m_leftPIDController = new PIDController(ShooterConstants.kP, ShooterConstants.kI,
            ShooterConstants.kD);
    private final PIDController m_rightPIDController = new PIDController(ShooterConstants.kP, ShooterConstants.kI,
            ShooterConstants.kD);
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(ShooterConstants.kS,
      ShooterConstants.kV, ShooterConstants.kA);

    public Shooter() {
        shooterPrimeLeft.restoreFactoryDefaults();  
        shooterPrimeRight.restoreFactoryDefaults();
        shooterPrimeLeft.setSmartCurrentLimit(ShooterConstants.currentLimit);
        shooterPrimeRight.setSmartCurrentLimit(ShooterConstants.currentLimit);

        m_leftPIDController.setTolerance(0, ShooterConstants.velocityTolerance);
        m_rightPIDController.setTolerance(0, ShooterConstants.velocityTolerance);
    }

    public void setShooterFeed(double speed) {
        shooterTopFeed.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public void set(double setpoint) {
        m_leftPIDController.reset();
        m_rightPIDController.reset();

        double feedforward = m_feedforward.calculate(setpoint);
        double leftOutput = m_leftPIDController.calculate(encoderLeft.getVelocity(), setpoint);
        double rightOutput = m_rightPIDController.calculate(encoderRight.getVelocity(), setpoint);
        
        shooterPrimeLeft.setVoltage(leftOutput + feedforward);
        shooterPrimeRight.setVoltage(-(rightOutput + feedforward));
    }   

    public boolean atSetpoint() {
        return m_leftPIDController.atSetpoint() && m_rightPIDController.atSetpoint();
    }

    public void extend() {
    }

    public void stop() {
        shooterTopFeed.set(VictorSPXControlMode.PercentOutput, 0);
        shooterPrimeRight.stopMotor();
        shooterPrimeLeft.stopMotor();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter Encoder", encoderLeft.getVelocity());
        if (atSetpoint() && !DriverStation.isAutonomous()){
            RobotContainer.controllers.mControls.getHID().setRumble(RumbleType.kBothRumble, 0.3);
        }
        else {
            RobotContainer.controllers.mControls.getHID().setRumble(RumbleType.kBothRumble, 0);
        }
        SmartDashboard.putBoolean("At Max RPM?", atSetpoint());
    }
}

