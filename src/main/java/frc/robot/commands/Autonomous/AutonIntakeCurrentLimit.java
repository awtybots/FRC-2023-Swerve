/** Thank you GOFIRST-Robotics! */
package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.IntakeMech;

public class AutonIntakeCurrentLimit extends CommandBase {

    private final IntakeMech s_intake;

    /** Command to use Gyro data to resist the tip angle from the beam - to stabilize and balance. */
    public AutonIntakeCurrentLimit(IntakeMech s_intake) {
        addRequirements(s_intake);
        this.s_intake = s_intake;
    }

    @Override
    public void execute() {
        boolean isCone = RobotContainer.coneModeEnabled();
        s_intake.intake(isCone ? -0.9 : 0.9, true);
    }

    @Override
    public void end(boolean interrupted) {
        s_intake.intake(0, false);
    }
}
