package frc.robot.commands.Autonomous.Balance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve.Swerve;

public class Balance extends SequentialCommandGroup {

    public Balance(
            Swerve s_Swerve) {
        addRequirements(s_Swerve);
        addCommands(
                new QuickBalance(s_Swerve),
                new WaitCommand(1.5),
                new PreciseBalance(s_Swerve)
        );
    }
}
