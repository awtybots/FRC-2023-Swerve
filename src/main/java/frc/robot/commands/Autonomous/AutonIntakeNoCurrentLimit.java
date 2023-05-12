package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MechanicalParts.IntakeMech;

public class AutonIntakeNoCurrentLimit extends CommandBase {

    private final IntakeMech s_intake;
    private boolean isCone = false;

    public AutonIntakeNoCurrentLimit(IntakeMech IntakeSubsystem) {
        addRequirements(IntakeSubsystem);
        this.s_intake = IntakeSubsystem;
    }

    public AutonIntakeNoCurrentLimit(IntakeMech IntakeSubsystem, boolean isCone) {
        addRequirements(IntakeSubsystem);
        this.s_intake = IntakeSubsystem;
        this.isCone = isCone;
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // boolean isCone = RobotContainer.getIsCone();
        s_intake.intake(isCone ? 0.7 : -0.7, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
