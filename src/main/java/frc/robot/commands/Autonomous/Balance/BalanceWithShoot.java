package frc.robot.commands.Autonomous.Balance;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.State;
import frc.robot.commands.Autonomous.ShootPiece.ShootPiece;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;
import frc.robot.subsystems.MechanicalParts.IntakeMech;
import frc.robot.subsystems.Swerve.Swerve;

public class BalanceWithShoot extends SequentialCommandGroup {

    public BalanceWithShoot(
            Swerve s_Swerve,
            LedSubsystem s_Led,
            ClawSubsystem s_Claw,
            ArmMech s_Arm,
            ElevatorMech s_ElevatorSubsystem,
            IntakeMech s_Intake) {
        addRequirements(s_Swerve);
        addCommands(
                new InstantCommand(() -> RobotContainer.setCurrentState(State.Balance)),
                // new QuickBalance(s_Swerve, s_Led),
                // new WaitCommand(1.5),
                new ShootPiece(s_Intake, s_ElevatorSubsystem, s_Arm, s_Claw),
                new PreciseBalance(s_Swerve, s_Led));
    }
}
