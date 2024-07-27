package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.HangConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ampShoot;
import frc.robot.commands.floorIntake;
import frc.robot.commands.floorReverse;
import frc.robot.commands.hangRetract;
import frc.robot.commands.pointAndShoot;
import frc.robot.commands.prime;
import frc.robot.commands.topIntake;
import frc.robot.autonCommands.primeNshoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
        private final Field2d field;
        private final Drivetrain mDrivetrain = new Drivetrain();
        private final Intake mIntake = new Intake();
        private final Shooter mShooter = new Shooter();
        private final Hang mHang = new Hang();

        private final SendableChooser<Command> autoChooser;
        public static class controllers{
                public final static CommandXboxController mDriver = new CommandXboxController(OperatorConstants.driverPort);
                public final static CommandXboxController mControls = new CommandXboxController(OperatorConstants.controlsPort);
        }

        public RobotContainer() {
        field = new Field2d();
        SmartDashboard.putData("Field", field);

        // Logging callback for current robot pose
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.setRobotPose(pose);
        });

        // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            field.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            field.getObject("path").setPoses(poses);
        });
        
        NamedCommands.registerCommand("prime", new prime(mShooter));
        NamedCommands.registerCommand("primeNshoot", new primeNshoot(mShooter, mIntake));
        NamedCommands.registerCommand("floorIntake", new floorIntake(mIntake));
        NamedCommands.registerCommand("ampShoot", new ampShoot(mShooter, mIntake));
        configureBindings();
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
        
    }


    private void configureBindings() {
        // Tank Drive (single controller)
        //FOR DUAL CONTROLLER SETUP COMMENT THIS BLOCK AND UNCOMMENT THE NEXT BLOCK
        /*mDrivetrain.setDefaultCommand(
                new RunCommand(
                        () -> mDrivetrain.drive(controllers.mControls.getLeftY(), controllers.mControls.getRightY()),
                        mDrivetrain));*/

        /* FOR DUAL CONTROLLER SETUP UNCOMMENT THIS BLOCK*/
        //Tank Drive (2 controllers)            
        mDrivetrain.setDefaultCommand(new RunCommand(
                () -> mDrivetrain.drive(controllers.mDriver.getLeftY(), controllers.mDriver.getRightY()),
                mDrivetrain));

        // Amp Shoot
        controllers.mControls

                .b()
                .whileTrue(
                new ampShoot(mShooter, mIntake).withTimeout(0.5));

        // Shooter Prime
        controllers.mControls
                .leftTrigger()
                .whileTrue(
                        new prime(mShooter));

        // Shooter Launch
        controllers.mControls
                .leftBumper()
                .whileTrue(
                        new pointAndShoot(mShooter, mIntake));

        // Floor Intake
        controllers.mControls
                .rightTrigger()
                .whileTrue(
                        new floorIntake(mIntake));

        // Floor Intake Reverse
        controllers.mControls
                .rightBumper()
                .whileTrue(
                        new floorReverse(mIntake));

        // Top Intake
        controllers.mControls
                .x()
                .whileTrue(
                        new topIntake(mShooter));

        // Hang Winch
        controllers.mControls
                .a()
                .whileTrue(
                        new hangRetract(mHang, HangConstants.speed));

        // Hang Unwinch
        controllers.mControls
                .y()
                .whileTrue(
                        new hangRetract(mHang, -HangConstants.speed));
        
        /*
        controllers.mControls
        .a()
        .and(controllers.mControls.rightBumper())
        .whileTrue(mDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        controllers.mControls
                .b()
                .and(controllers.mControls.rightBumper())
                .whileTrue(mDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        controllers.mControls
                .x()
                .and(controllers.mControls.rightBumper())
                .whileTrue(mDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        controllers.mControls
                .y()
                .and(controllers.mControls.rightBumper())
                .whileTrue(mDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        controllers.mControls
                .a()
                .and(controllers.mControls.leftBumper())
                .whileTrue(mDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        controllers.mControls
                .b()
                .and(controllers.mControls.leftBumper())
                .whileTrue(mDrivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        controllers.mControls
                .x()
                .and(controllers.mControls.leftBumper())
                .whileTrue(mDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        controllers.mControls
                .y()
                .and(controllers.mControls.leftBumper())
                .whileTrue(mDrivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));
                */
    }

    public Command getAutonomousCommand() {
        mDrivetrain.resetEncoders();
        mDrivetrain.resetGyro();
        return autoChooser.getSelected();
    }
}

