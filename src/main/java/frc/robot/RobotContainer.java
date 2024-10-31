package frc.robot;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
        public final static CommandXboxController mDriver = new CommandXboxController(0);
        public final static Intake m_intake = new Intake();
        public final static Shooter m_shooter = new Shooter();
        public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        
    }
}
