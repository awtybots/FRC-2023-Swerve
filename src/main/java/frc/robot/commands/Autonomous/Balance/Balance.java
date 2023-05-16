package frc.robot.commands.Autonomous.Balance;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.State;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class Balance extends SequentialCommandGroup {

    public Balance(Swerve s_Swerve, LedSubsystem s_Led) {
        addRequirements(s_Swerve);
        addCommands(
                new InstantCommand(() -> RobotContainer.setCurrentState(State.Balance)),
                new PreciseBalance(s_Swerve));
    }
}
