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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Amp;
import frc.robot.commands.FloorIntake;
import frc.robot.commands.HangRetract;
import frc.robot.commands.Shoot;
import frc.robot.commands.SnapSpeaker;
import frc.robot.commands.SpeakerAlign;
import frc.robot.commands.Prime;
import frc.robot.commands.Rotate;
import frc.robot.commands.TopIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
        public final Drivetrain m_drivetrain = new Drivetrain();
        private final Intake m_intake = new Intake();
        private final Shooter m_shooter = new Shooter();
        private final Hang m_hang = new Hang();

        private final SendableChooser<Command> autoChooser;
        public static class controllers{
                public final static CommandXboxController mDriver = new CommandXboxController(OperatorConstants.driverPort);
                public final static CommandXboxController mControls = new CommandXboxController(OperatorConstants.controlsPort);
        }

        public RobotContainer() {
        NamedCommands.registerCommand("Prime", new Prime(m_shooter));
        NamedCommands.registerCommand("Shoot", new Shoot(m_shooter, m_intake));
        NamedCommands.registerCommand("Floor Intake", new FloorIntake(m_intake, 1));
        NamedCommands.registerCommand("Amp", new Amp(m_shooter, m_intake));
        configureBindings();
        
        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }


    private void configureBindings() {
        // Tank Drive (single controller)
        //FOR DUAL CONTROLLER SETUP COMMENT THIS BLOCK AND UNCOMMENT THE NEXT BLOCK
        /*m_drivetrain.setDefaultCommand(
                new RunCommand(
                        () -> m_drivetrain.drive(controllers.mControls.getLeftY(), controllers.mControls.getRightY()),
                        m_drivetrain));*/

        /* FOR DUAL CONTROLLER SETUP UNCOMMENT THIS BLOCK*/
        //Tank Drive (2 controllers)            
        m_drivetrain.setDefaultCommand(new RunCommand(
                () -> m_drivetrain.drive(controllers.mDriver.getLeftY(), controllers.mDriver.getRightY()),
                m_drivetrain));        

        // Amp Shoot
        controllers.mControls
                .b()
                .whileTrue(
                new Amp(m_shooter, m_intake).withTimeout(0.5));

        // Shooter Prime
        controllers.mControls
                .leftTrigger()
                .whileTrue(
                        new Prime(m_shooter));

        // Shooter Launch
        controllers.mControls
                .leftBumper()
                .whileTrue(
                        new Shoot(m_shooter, m_intake));

        // Floor Intake
        controllers.mControls
                .rightTrigger()
                .whileTrue(
                        new FloorIntake(m_intake, -IntakeConstants.speed));

        // Floor Intake Reverse
        controllers.mControls
                .rightBumper()
                .whileTrue(
                        new FloorIntake(m_intake, IntakeConstants.speed));

        // Top Intake
        controllers.mControls
                .x()
                .whileTrue(
                        new TopIntake(m_shooter));

        // Hang Winch
        controllers.mControls
                .a()
                .whileTrue(
                        new HangRetract(m_hang, HangConstants.speed));

        // Hang Unwinch
        controllers.mControls
                .y()
                .whileTrue(
                        new HangRetract(m_hang, -HangConstants.speed));

        // BUTTON 2 FOR SIM                        
        controllers.mDriver
                .b().onTrue(new SnapSpeaker(m_drivetrain));

        // BUTTON 3 FOR SIM
        controllers.mDriver
                .x().onTrue(new SpeakerAlign(m_drivetrain));
        
        
        controllers.mControls
        .a()
        .and(controllers.mControls.rightBumper())
        .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        controllers.mControls
                .b()
                .and(controllers.mControls.rightBumper())
                .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        controllers.mControls
                .x()
                .and(controllers.mControls.rightBumper())
                .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        controllers.mControls
                .y()
                .and(controllers.mControls.rightBumper())
                .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        controllers.mControls
                .a()
                .and(controllers.mControls.leftBumper())
                .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        controllers.mControls
                .b()
                .and(controllers.mControls.leftBumper())
                .whileTrue(m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        controllers.mControls
                .x()
                .and(controllers.mControls.leftBumper())
                .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));
        controllers.mControls
                .y()
                .and(controllers.mControls.leftBumper())
                .whileTrue(m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}

