package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Positions.Nodes.HighNodePosition.HighNodePosition;
import frc.robot.commands.Positions.Nodes.MidNodePosition;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MechanicalParts.ArmElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.IntakeSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class Place extends SequentialCommandGroup {

    public Place(
            Swerve s_Swerve,
            LimelightSubsystem s_Limelight,
            ClawSubsystem s_Claw,
            ArmElevatorSubsystem s_ArmElevator,
            ElevatorSubsystem s_Elevator,
            IntakeSubsystem s_Intake,
            int nodeId,
            boolean isCone) {
        addRequirements(s_Swerve, s_Limelight, s_Claw, s_ArmElevator, s_Elevator, s_Intake);
        addCommands(
                // new PlaceSetup(s_Swerve, s_Limelight, isCone),
                nodeId == 0
                        ? new MidNodePosition(s_Elevator, s_ArmElevator, s_Claw)
                        : new HighNodePosition(s_Elevator, s_ArmElevator, s_Claw),
                new AutonIntakeNoCurrentLimit(s_Intake, s_Limelight).withTimeout(0.3),
                new InstantCommand(() -> s_Intake.intake(0, false)),
                new StowPosition(s_Elevator, s_ArmElevator, s_Claw));
    }
}
