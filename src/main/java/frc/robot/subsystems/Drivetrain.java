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
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.cameraConstants;

import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.Encoder;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.PathPlannerLogging;
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
  public final PIDController m_roationalPIDController = new PIDController(0.015, 0,0.003)
     ;

  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriverConstants.kS,
      DriverConstants.kV, DriverConstants.kA);

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriverConstants.trackWidth);
  private final DifferentialDrivePoseEstimator m_poseEstimator = new DifferentialDrivePoseEstimator(
      kinematics,
      gyro.getRotation2d(),
      driveEncoderLeft.getDistance(),
      driveEncoderRight.getDistance(),
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

  public final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (Measure<Voltage> volts) -> {
            leftFront.set(volts.in(Volts) / RobotController.getBatteryVoltage());
            rightFront.set(volts.in(Volts) / RobotController.getBatteryVoltage());
          },
          null,
          this));

  public Field2d m_field = new Field2d();

  public List<Pose2d> posesPreAlignmentBlue = new ArrayList<>();
  public List<Pose2d> posesAlignedBlue = new ArrayList<>();

  public List<Pose2d> posesPreAlignmentRed = new ArrayList<>();
  public List<Pose2d> posesAlignedRed = new ArrayList<>();

  private final EncoderSim m_leftEncoderSim = new EncoderSim(driveEncoderLeft);
  private final EncoderSim m_rightEncoderSim = new EncoderSim(driveEncoderRight);
  private final LinearSystem<N2, N2, N2> m_drivetrainSystem =
      LinearSystemId.identifyDrivetrainSystem(DriverConstants.kV, DriverConstants.kA, DriverConstants.kV, DriverConstants.kA);
  private final DifferentialDrivetrainSim m_driveSim =
      new DifferentialDrivetrainSim(
          m_drivetrainSystem, DCMotor.getNEO(4), 8.46, DriverConstants.trackWidth, Units.inchesToMeters(DriverConstants.wheelDiameter/2), null);

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

    m_leftEncoderSim.setDistancePerPulse(DriverConstants.distancePerPulse);
    m_rightEncoderSim.setDistancePerPulse(DriverConstants.distancePerPulse);

    m_roationalPIDController.enableContinuousInput(-180, 180);
    m_roationalPIDController.setTolerance(DriverConstants.rotationalTolerance);

    AutoBuilder.configureLTV(
        this::getPose,
        this::resetPose,
        this::getChassisSpeeds,
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
    photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cam,
        robotToCam);

    posesPreAlignmentBlue.add(new Pose2d(2, 5.55, Rotation2d.fromDegrees(0)));
    posesPreAlignmentBlue.add(new Pose2d(1, 6.97, Rotation2d.fromDegrees(-116)));
    posesPreAlignmentBlue.add(new Pose2d(1, 4.02, Rotation2d.fromDegrees(116)));

    posesAlignedBlue.add(new Pose2d(1.31, 5.56, Rotation2d.fromDegrees(0)));
    posesAlignedBlue.add(new Pose2d(0.74, 6.61, Rotation2d.fromDegrees(64)));
    posesAlignedBlue.add(new Pose2d(0.74, 4.50, Rotation2d.fromDegrees(-64)));

    for(int i = 0; i < 3; i++) {
      posesPreAlignmentRed.add(GeometryUtil.flipFieldPose(posesPreAlignmentBlue.get(i)));
      posesAlignedRed.add(GeometryUtil.flipFieldPose(posesAlignedBlue.get(i)));
    }

    SmartDashboard.putData("Field", m_field);
    SmartDashboard.putData("LEFT ENCODER", driveEncoderLeft);
    SmartDashboard.putData("RIGHT ENCODER", driveEncoderRight);
    SmartDashboard.putData("Drivetrain", drivetrain);
    SmartDashboard.putNumber("Manual X", getPose().getX());
    SmartDashboard.putNumber("Manual Y", getPose().getY());
    SmartDashboard.putNumber("Manual Angle", getHeadingRelative());
    SmartDashboard.putBoolean("Manually Reset Pose?", false);
  }

  public void drive(double left, double right) {
    drivetrain.tankDrive(left, right, true);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Gyro", getHeadingRelative());
    SmartDashboard.putNumber("Position Error", m_roationalPIDController.getPositionError());
    m_poseEstimator.update(gyro.getRotation2d(), driveEncoderLeft.getDistance(), driveEncoderRight.getDistance());
    
    var visionEST = photonPoseEstimator.update();
    visionEST.ifPresent(
        est -> {
          m_poseEstimator.addVisionMeasurement(
              est.estimatedPose.toPose2d(), est.timestampSeconds);
          SmartDashboard.putBoolean("vision", true);
        });

    if (SmartDashboard.getBoolean("Manually Reset Pose?", false)) {
      double x = SmartDashboard.getNumber("Manual X", getPose().getX());
      double y = SmartDashboard.getNumber("Manual Y", getPose().getY());
      double angle = SmartDashboard.getNumber("Manual Angle", getHeadingRelative());
      resetPose(new Pose2d(x, y, new Rotation2d(Math.toRadians(angle))));
    }
    
    logPathPlanner();  

    m_field.setRobotPose(getPose());
  }

  public void logPathPlanner() {
    PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
        m_field.setRobotPose(pose);
        resetPose(pose);
    });
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
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetGyro() {
    gyro.reset();
  }
  
  public void curvatureDrive(double xSpeed, double rotation, boolean turnInPlace) {
    drivetrain.curvatureDrive(xSpeed, rotation, turnInPlace);
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

  public Pose2d nearestAutoAlign(int alignmentType) {
    Optional<Alliance> ally = DriverStation.getAlliance();
    if (ally.isPresent()) {
      if (ally.get() == Alliance.Red) {
        return getPose().nearest(alignmentType == 0 ? posesPreAlignmentRed : posesAlignedRed);
      }
    }

    return getPose().nearest(alignmentType == 0 ? posesPreAlignmentBlue : posesAlignedBlue);
  }

  public double alignmentAngle(Pose2d target, Pose2d current) {
    return Math.toDegrees(Math.atan2(target.getY() - current.getY(), target.getX() - current.getX()));
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

  public double getHeadingRelative() {
    // return ;
    SmartDashboard.putNumber("360000 output", Math.abs((gyro.getRotation2d().getDegrees()+360000) % 360));
    SmartDashboard.putNumber("IEEE Remainder Output", Math.IEEEremainder(gyro.getAngle(), 360) * -1);
    return Math.IEEEremainder(gyro.getAngle(), 360) * -1;
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

  public void simUpdate() {
    m_driveSim.setInputs(leftFront.get() * RobotController.getInputVoltage(),
                         rightFront.get() * RobotController.getInputVoltage());

    m_driveSim.update(0.02);
  
    m_leftEncoderSim.setDistance(m_driveSim.getLeftPositionMeters());
    m_leftEncoderSim.setRate(m_driveSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderSim.setDistance(m_driveSim.getRightPositionMeters());
    m_rightEncoderSim.setRate(m_driveSim.getRightVelocityMetersPerSecond());
    int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
    angle.set(-m_driveSim.getHeading().getDegrees());
      }
  

}