package frc.robot.commands.Autonomous.ShootPiece;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autonomous.AutonIntakeNoCurrentLimit;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.MechanicalParts.ArmElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.IntakeSubsystem;

public class ShootPiece extends SequentialCommandGroup {

    public ShootPiece(
            IntakeSubsystem s_Intake,
            ElevatorSubsystem s_Elevator,
            ArmElevatorSubsystem s_ArmElevator,
            ClawSubsystem s_Claw) {
        addRequirements(s_Intake, s_Elevator, s_ArmElevator, s_Claw);
        addCommands(
                new Position(s_ArmElevator, s_Elevator, s_Claw),
                new AutonIntakeNoCurrentLimit(s_Intake).withTimeout(0.3),
                new InstantCommand(() -> s_Intake.intake(0, false)),
                new StowPosition(s_Elevator, s_ArmElevator, s_Claw));
    }
}
