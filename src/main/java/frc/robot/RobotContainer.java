package frc.robot;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.HangConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FloorIntake;
import frc.robot.commands.HangRetract;
import frc.robot.commands.Shoot;
import frc.robot.commands.Prime;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hang;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
        public final Drivetrain m_drivetrain = new Drivetrain();
        private final Intake m_intake = new Intake();
        private final Shooter m_shooter = new Shooter();
        private final Hang m_hang = new Hang();

        public static class controllers{
                public final static CommandXboxController mDriver = new CommandXboxController(OperatorConstants.driverPort);
                public final static CommandXboxController mControls = new CommandXboxController(OperatorConstants.controlsPort);
        }

        public RobotContainer() {
        configureBindings();
    }


    private void configureBindings() {
        //Tank Drive (2 controllers)            
        m_drivetrain.setDefaultCommand(new RunCommand(
                () -> m_drivetrain.drive(controllers.mDriver.getLeftY(), controllers.mDriver.getRightY()),
                m_drivetrain));        

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
    }
}

