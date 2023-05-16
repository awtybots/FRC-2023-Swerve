package frc.robot.commands.Autonomous.Balance;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.State;
import frc.robot.commands.Autonomous.ShootPiece.ShootPiece;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;
import frc.robot.subsystems.MechanicalParts.IntakeMech;
import frc.robot.subsystems.Swerve.Swerve;

public class BalanceWithShoot extends SequentialCommandGroup {

    public BalanceWithShoot(
            Swerve s_swerve,
            ClawSubsystem s_claw,
            ArmMech s_arm,
            ElevatorMech s_elevator,
            IntakeMech s_intake) {
        addRequirements(s_swerve);
        addCommands(
                new InstantCommand(() -> RobotContainer.setCurrentState(State.Balance)),
                new ShootPiece(s_intake, s_elevator, s_arm, s_claw),
                new PreciseBalance(s_swerve));
    }
}
