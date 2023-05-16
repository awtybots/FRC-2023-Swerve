package frc.robot.commands.Autonomous.ShootPiece;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Autonomous.AutonIntakeNoCurrentLimit;
import frc.robot.commands.Positions.StowPos;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;
import frc.robot.subsystems.MechanicalParts.IntakeMech;

public class ShootPiece extends SequentialCommandGroup {

    public ShootPiece(
            IntakeMech s_intake, ElevatorMech s_elevator, ArmMech s_arm, ClawSubsystem s_claw) {
        addRequirements(s_intake, s_elevator, s_arm, s_claw);
        addCommands(
                new Position(s_arm, s_elevator, s_claw),
                new WaitCommand(0.3),
                new AutonIntakeNoCurrentLimit(s_intake).withTimeout(0.3),
                new InstantCommand(() -> s_intake.intake(0, false)),
                new StowPos(s_elevator, s_arm, s_claw));
    }
}
