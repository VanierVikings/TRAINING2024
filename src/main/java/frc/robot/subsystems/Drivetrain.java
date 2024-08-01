// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.cameraConstants;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Encoder;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;

public class Drivetrain extends SubsystemBase {
  private final CANSparkBase leftRear = new CANSparkMax(DriverConstants.leftRearId, MotorType.kBrushless);
  private final CANSparkBase leftFront = new CANSparkMax(DriverConstants.leftFrontId, MotorType.kBrushless);
  private final CANSparkBase rightRear = new CANSparkMax(DriverConstants.rightRearId, MotorType.kBrushless);
  private final CANSparkBase rightFront = new CANSparkMax(DriverConstants.rightFrontId, MotorType.kBrushless);
  private final DifferentialDrive drivetrain = new DifferentialDrive(leftFront, rightFront);

  private PhotonCamera cam = new PhotonCamera(cameraConstants.kCamName);
  private AprilTagFieldLayout aprilTagFieldLayout;
  private Transform3d robotToCam;
  private PhotonPoseEstimator photonPoseEstimator;

  private final RelativeEncoder encoderLeftFront = leftFront.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
  private final RelativeEncoder encoderRightFront = rightFront.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

  private final RelativeEncoder encoderLeftRear = leftRear.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);
  private final RelativeEncoder encoderRightRear = rightRear.getEncoder(SparkRelativeEncoder.Type.kHallSensor, 42);

  private final Encoder driveEncoderLeft = new Encoder(0, 1, true);
  private final Encoder driveEncoderRight = new Encoder(2, 3, false);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);

  private final PIDController m_leftPIDController = new PIDController(DriverConstants.kP, DriverConstants.kI,
      DriverConstants.kD);
  private final PIDController m_rightPIDController = new PIDController(DriverConstants.kP, DriverConstants.kI,
      DriverConstants.kD);
  private final PIDController m_roationalPIDController = new PIDController(DriverConstants.kP, DriverConstants.kI,
      DriverConstants.kD);

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriverConstants.kS,
      DriverConstants.kV, DriverConstants.kA);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriverConstants.trackWidth);

  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));
  private final DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(
      kinematics,
      gyro.getRotation2d(),
      driveEncoderLeft.getDistance(),
      driveEncoderRight.getDistance(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (Measure<Voltage> volts) -> {
            leftFront.set(volts.in(Volts) / RobotController.getBatteryVoltage());
            rightFront.set(volts.in(Volts) / RobotController.getBatteryVoltage());
          },
          log -> {
            log.motor("drive-left")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        leftFront.getAppliedOutput() * leftFront.getBusVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(driveEncoderLeft.getDistance(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(driveEncoderLeft.getRate(), MetersPerSecond));
            log.motor("drive-right")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        rightFront.getAppliedOutput() * rightFront.getBusVoltage(), Volts))
                .linearPosition(m_distance.mut_replace(driveEncoderRight.getDistance(), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(driveEncoderRight.getRate(), MetersPerSecond));
          },
          this));

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

    leftFront.setIdleMode(IdleMode.kCoast);
    leftRear.setIdleMode(IdleMode.kCoast);
    rightFront.setIdleMode(IdleMode.kCoast);
    rightRear.setIdleMode(IdleMode.kCoast);

    encoderLeftFront.setPositionConversionFactor(-DriverConstants.positionConversionFactor);
    encoderLeftFront.setVelocityConversionFactor(-DriverConstants.velocityConversionFactor);

    encoderRightFront.setPositionConversionFactor(DriverConstants.positionConversionFactor);
    encoderRightFront.setVelocityConversionFactor(DriverConstants.velocityConversionFactor);

    encoderLeftRear.setPositionConversionFactor(-DriverConstants.positionConversionFactor);
    encoderLeftRear.setVelocityConversionFactor(-DriverConstants.velocityConversionFactor);

    encoderRightRear.setPositionConversionFactor(DriverConstants.positionConversionFactor);
    encoderRightRear.setVelocityConversionFactor(DriverConstants.velocityConversionFactor);

    driveEncoderLeft.setDistancePerPulse(DriverConstants.distancePerPulse);
    driveEncoderRight.setDistancePerPulse(DriverConstants.distancePerPulse);

    m_roationalPIDController.enableContinuousInput(180, 180);

    AutoBuilder.configureLTV(
        this::getPose,
        this::resetPose,
        this::giveCurrentSpeeds,
        this::chassisDrive,
        0.02,
        new ReplanningConfig(),
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0), new Rotation3d(0, 10, 0));
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam,
        robotToCam);
  }

  public void drive(double left, double right) {
    drivetrain.tankDrive(left, right, true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("LEFT ENCODER", driveEncoderLeft);
    SmartDashboard.putData("RIGHT ENCODER", driveEncoderRight);
    SmartDashboard.putData("Drivetrain", drivetrain);
    SmartDashboard.putData("Gyro", gyro);

    m_poseEstimator.update(gyro.getRotation2d(), driveEncoderLeft.getDistance(), driveEncoderRight.getDistance());

    var visionEST = photonPoseEstimator.update();
    visionEST.ifPresent(
        est -> {
          m_poseEstimator.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds);
        });
  }

  public ChassisSpeeds giveCurrentSpeeds() {
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(driveEncoderLeft.getRate(), driveEncoderRight.getRate());
    ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(wheelSpeeds);
    return chassisSpeeds;
  }

  public void resetEncoders() {
    driveEncoderLeft.reset();
    driveEncoderRight.reset();
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void resetPose(Pose2d pose) {
    gyro.setAngleAdjustment(pose.getRotation().getDegrees());
    m_poseEstimator.resetPosition(
        gyro.getRotation2d(), driveEncoderLeft.getDistance(), driveEncoderRight.getDistance(), pose);
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    double leftOutput = m_leftPIDController.calculate(driveEncoderLeft.getRate(), speeds.leftMetersPerSecond);
    double rightOutput = m_rightPIDController.calculate(driveEncoderRight.getRate(), speeds.rightMetersPerSecond);

    leftFront.setVoltage(-(leftOutput + leftFeedforward));
    rightFront.setVoltage(-(rightOutput + rightFeedforward));
  }

  public Pose2d nearestAutoAlign() {
    List<Pose2d> poses = new ArrayList<Pose2d>();
    Pose2d speakerCenter = new Pose2d(1.30, 5.56, Rotation2d.fromDegrees(0));
    Pose2d speakerAmp = new Pose2d(0.74, 6.59, Rotation2d.fromDegrees(64));
    Pose2d speakerSource = new Pose2d(0.74, 4.49, Rotation2d.fromDegrees(-64.00));

    poses.add(speakerCenter);
    poses.add(speakerSource);
    poses.add(speakerAmp);

    return getPose().nearest(poses);
  }

  public void rotate(double angle) {
    // arade implementation
    double angleFeedforward = m_feedforward.calculate(angle);
    double rotationalOutput = m_roationalPIDController.calculate(getHeading(), angle);
    drivetrain.arcadeDrive(0, rotationalOutput + angleFeedforward);
  }

  public void chassisDrive(ChassisSpeeds speed) {
    setSpeeds(kinematics.toWheelSpeeds(speed));
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(driveEncoderLeft.getRate(), driveEncoderRight.getRate());
  }

  public void setMaxOutput(double maxOutput) {
    drivetrain.setMaxOutput(maxOutput);
  }

  public Encoder getLeftEncoder() {
    return driveEncoderLeft;
  }

  public Encoder getRightEncoder() {
    return driveEncoderRight;
  }

  public double getHeading() {
    return gyro.getRotation2d().getDegrees();
  }

  public void stop() {
    leftFront.stopMotor();
    rightRear.stopMotor();
    rightFront.stopMotor();
    leftRear.stopMotor();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction);
  }

}
