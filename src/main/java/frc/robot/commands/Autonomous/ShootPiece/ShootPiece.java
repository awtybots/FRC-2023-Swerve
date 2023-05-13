package frc.robot.commands.Autonomous.ShootPiece;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Autonomous.AutonIntakeNoCurrentLimit;
import frc.robot.commands.Positions.StowPos;
import frc.robot.subsystems.MechanicalParts.ArmElevatorMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;
import frc.robot.subsystems.MechanicalParts.IntakeMech;

public class ShootPiece extends SequentialCommandGroup {

    public ShootPiece(
            IntakeMech s_Intake,
            ElevatorMech s_Elevator,
            ArmElevatorMech s_ArmElevator,
            ClawSubsystem s_Claw) {
        addRequirements(s_Intake, s_Elevator, s_ArmElevator, s_Claw);
        addCommands(
                new Position(s_ArmElevator, s_Elevator, s_Claw),
                new WaitCommand(0.3),
                new AutonIntakeNoCurrentLimit(s_Intake).withTimeout(0.3),
                new InstantCommand(() -> s_Intake.intake(0, false)),
                new StowPos(s_Elevator, s_ArmElevator, s_Claw));
    }
}
