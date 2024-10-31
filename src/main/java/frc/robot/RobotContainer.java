package frc.robot;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
        public final static CommandXboxController mDriver = new CommandXboxController(0);
        public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        
    }
}
