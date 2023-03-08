/** Thank you GOFIRST-Robotics! */
package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MechanicalParts.IntakeSubsystem;

public class Intake extends CommandBase {

    private final IntakeSubsystem s_Intake;
    private final boolean isCone;

    /** Command to use Gyro data to resist the tip angle from the beam - to stabilize and balance. */
    public Intake(IntakeSubsystem s_Intake, boolean isCone) {
        addRequirements(s_Intake);
        this.s_Intake = s_Intake;
        this.isCone = isCone;
    }

    @Override
    public void execute() {
        s_Intake.intake(isCone ? -1 : 1, true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_Intake.intake(0, false);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return s_Intake.getOutputCurrent() > 15;
    }
}
