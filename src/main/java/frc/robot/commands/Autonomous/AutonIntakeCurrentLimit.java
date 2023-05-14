/** Thank you GOFIRST-Robotics! */
package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MechanicalParts.IntakeMech;

public class AutonIntakeCurrentLimit extends CommandBase {

    private final IntakeMech s_Intake;
    private boolean isCone;

    /** Command to use Gyro data to resist the tip angle from the beam - to stabilize and balance. */
    public AutonIntakeCurrentLimit(IntakeMech s_Intake) {
        addRequirements(s_Intake);
        isCone = RobotContainer.coneModeEnabled();
        this.s_Intake = s_Intake;
    }

    @Override
    public void execute() {
        isCone = RobotContainer.coneModeEnabled();
        s_Intake.intake(isCone ? -0.9 : 0.9, true);
    }

    @Override
    public void end(boolean interrupted) {
        s_Intake.intake(0, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
