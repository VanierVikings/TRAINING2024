// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriverConstants;

import edu.wpi.first.wpilibj.Encoder;

public class Drivetrain extends SubsystemBase {
  private final CANSparkMax leftRear = new CANSparkMax(DriverConstants.leftRearId, MotorType.kBrushless);
  private final CANSparkMax leftFront = new CANSparkMax(DriverConstants.leftFrontId, MotorType.kBrushless);
  private final CANSparkMax rightRear = new CANSparkMax(DriverConstants.rightRearId, MotorType.kBrushless);
  private final CANSparkMax rightFront = new CANSparkMax(DriverConstants.rightFrontId, MotorType.kBrushless);
  private final DifferentialDrive drivetrain = new DifferentialDrive(leftFront, rightFront);

  private final Encoder driveEncoderLeft = new Encoder(0, 1, true);
  private final Encoder driveEncoderRight = new Encoder(2, 3, false);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriverConstants.trackWidth);
  private final DifferentialDriveOdometry m_poseEstimator = new DifferentialDriveOdometry(gyro.getRotation2d(), driveEncoderLeft.getDistance(), driveEncoderRight.getDistance());


  public Drivetrain() {
    resetEncoders();
    gyro.reset();

    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    leftFront.setInverted(true);
    rightFront.setInverted(false);

    leftFront.setSmartCurrentLimit(DriverConstants.currentLimit);
    leftRear.setSmartCurrentLimit(DriverConstants.currentLimit);
    rightFront.setSmartCurrentLimit(DriverConstants.currentLimit);
    rightRear.setSmartCurrentLimit(DriverConstants.currentLimit);

    leftFront.setIdleMode(IdleMode.kBrake);
    leftRear.setIdleMode(IdleMode.kBrake);
    rightFront.setIdleMode(IdleMode.kBrake);
    rightRear.setIdleMode(IdleMode.kBrake);

    driveEncoderLeft.setDistancePerPulse(DriverConstants.distancePerPulse);
    driveEncoderRight.setDistancePerPulse(DriverConstants.distancePerPulse);

    SmartDashboard.putData("LEFT ENCODER", driveEncoderLeft);
    SmartDashboard.putData("RIGHT ENCODER", driveEncoderRight);
    SmartDashboard.putData("Drivetrain", drivetrain);
  }

  public void drive(double left, double right) {
    drivetrain.tankDrive(left, right, true);
  } 

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro", getHeading());
    m_poseEstimator.update(gyro.getRotation2d(), driveEncoderLeft.getDistance(), driveEncoderRight.getDistance());
}

  public ChassisSpeeds getChassisSpeeds() {
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(driveEncoderLeft.getRate(), driveEncoderRight.getRate());
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
    return chassisSpeeds;
  }

  public void resetEncoders() {
    driveEncoderLeft.reset();
    driveEncoderRight.reset();
  }

  public Pose2d getPose() {
    return m_poseEstimator.getPoseMeters();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void resetPose(Pose2d pose) {
    gyro.setAngleAdjustment(pose.getRotation().getDegrees());
    m_poseEstimator.resetPosition(
        gyro.getRotation2d(), driveEncoderLeft.getDistance(), driveEncoderRight.getDistance(), pose);
  }
  
  public double getHeading() {
    return m_poseEstimator.getPoseMeters().getRotation().getDegrees();
  }

  public void stop() {
    leftFront.stopMotor();
    rightRear.stopMotor();
    rightFront.stopMotor();
    leftRear.stopMotor();
  }
}