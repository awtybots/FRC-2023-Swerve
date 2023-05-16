package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MechanicalParts.IntakeMech;

public class AutonIntakeNoCurrentLimit extends CommandBase {

    private final IntakeMech s_intake;
    private final boolean isCone;

    public AutonIntakeNoCurrentLimit(IntakeMech s_intake) {
        this(s_intake, false);
    }

    public AutonIntakeNoCurrentLimit(IntakeMech IntakeSubsystem, boolean isCone) {
        addRequirements(IntakeSubsystem);
        this.s_intake = IntakeSubsystem;
        this.isCone = isCone;
    }

    @Override
    public void execute() {
        s_intake.intake(isCone ? 0.7 : -0.7, true);
    }
}
