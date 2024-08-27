package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
    public boolean active;
    public double LeftRPM, RightRPM;
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
        shooterPrimeLeft.setIdleMode(IdleMode.kBrake);
        shooterPrimeRight.setIdleMode(IdleMode.kBrake);

        m_rightPIDController.setTolerance(ShooterConstants.tolerance);
        m_leftPIDController.setTolerance(ShooterConstants.tolerance, 10);

    }

    public void setShooterFeed(double speed) {  
        shooterTopFeed.set(VictorSPXControlMode.PercentOutput, speed);
    }

    public void set(double setpoint) {
        

        double feedforward = m_feedforward.calculate(setpoint);
        m_leftPIDController.setSetpoint(setpoint);
        m_rightPIDController.setSetpoint(setpoint);
        double leftOutput = MathUtil.clamp(m_leftPIDController.calculate(Math.abs(LeftRPM), setpoint), 0,100);
        double rightOutput = MathUtil.clamp(m_rightPIDController.calculate(Math.abs(RightRPM), setpoint-200),0,100);
        SmartDashboard.putNumber("Setpoint", setpoint);
        SmartDashboard.putNumber("Left PID erorr", m_leftPIDController.getPositionError());
        SmartDashboard.putNumber("LEFT output", leftOutput);
        SmartDashboard.putNumber("Right output", rightOutput);
        SmartDashboard.putNumber("feedforward", feedforward);
        shooterPrimeLeft.setVoltage((-(leftOutput + feedforward))*RobotController.getBatteryVoltage());
        shooterPrimeRight.setVoltage((rightOutput + feedforward)*RobotController.getBatteryVoltage());
        if (atSetpoint() && !DriverStation.isAutonomous()){
            RobotContainer.controllers.mControls.getHID().setRumble(RumbleType.kBothRumble, 0.3);
        }
        else {
            RobotContainer.controllers.mControls.getHID().setRumble(RumbleType.kBothRumble, 0);
        }
    }   

    public boolean atSetpoint() {
        if (Math.abs(m_leftPIDController.getPositionError()) < 50){
            return true;
        } else{
            return false;
        }
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
        LeftRPM = encoderLeft.getVelocity();
        RightRPM = encoderRight.getVelocity();
        SmartDashboard.putNumber("LEFT Shooter Encoder",LeftRPM);
        SmartDashboard.putNumber("RIGHT Shooter Encoder", RightRPM);
        SmartDashboard.putBoolean("At Max RPM?", atSetpoint());
    }
}

