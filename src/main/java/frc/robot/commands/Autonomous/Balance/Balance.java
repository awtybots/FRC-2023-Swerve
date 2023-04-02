package frc.robot.commands.Autonomous.Balance;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.State;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class Balance extends SequentialCommandGroup {

    public Balance(Swerve s_Swerve, LedSubsystem s_Led) {
        addRequirements(s_Swerve);
        addCommands(
                new InstantCommand(() -> RobotContainer.setCurrentState(State.Balance)),
                new QuickBalance(s_Swerve, s_Led),
                new WaitCommand(1.5),
                new PreciseBalance(s_Swerve, s_Led));
    }
}
