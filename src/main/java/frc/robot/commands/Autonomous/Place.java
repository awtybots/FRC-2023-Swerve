package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Positions.Nodes.HighNodePosition.HighNodePosition;
import frc.robot.commands.Positions.Nodes.MidNodePosition;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.IntakeSubsystem;
import frc.robot.subsystems.MechanicalParts.PistonSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class Place extends SequentialCommandGroup {

    public Place(
            Swerve s_Swerve,
            LimelightSubsystem s_Limelight,
            ClawSubsystem s_Claw,
            ArmSubsystem s_Arm,
            ElevatorSubsystem s_Elevator,
            IntakeSubsystem s_Intake,
            PistonSubsystem s_Piston,
            int nodeId,
            boolean isCone) {
        addRequirements(s_Swerve, s_Limelight, s_Claw, s_Arm, s_Elevator, s_Intake);
        // ! only isCone = true done, not finished cube and possible intake?
        addCommands(
                // new PlaceSetup(s_Swerve, s_Limelight, isCone),
                nodeId == 0
                        ? new MidNodePosition(s_Elevator, s_Arm, s_Claw, s_Limelight)
                        : new HighNodePosition(s_Elevator, s_Arm, s_Claw, s_Limelight),
                new InstantCommand(() -> s_Intake.intake(isCone ? 1 : 0, true)),
                new StowPosition(s_Elevator, s_Arm, s_Claw));
    }
}
