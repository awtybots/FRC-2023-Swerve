package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Positions.Nodes.HighNodePosition.HighNodePosition;
import frc.robot.commands.Positions.Nodes.MidNodePosition;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.MechanicalParts.ArmElevatorMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;
import frc.robot.subsystems.MechanicalParts.IntakeMech;
import frc.robot.subsystems.Swerve.Swerve;

public class Place extends SequentialCommandGroup {

    public static Place Cone(
            int nodeId,
            Swerve s_Swerve,
            LimelightSubsystem s_Limelight,
            ClawSubsystem s_Claw,
            ArmElevatorMech s_ArmElevator,
            ElevatorMech s_Elevator,
            IntakeMech s_Intake) {
        return new Place(
                nodeId, true, s_Swerve, s_Limelight, s_Claw, s_ArmElevator, s_Elevator, s_Intake);
    }

    public static Place Cube(
            int nodeId,
            Swerve s_Swerve,
            LimelightSubsystem s_Limelight,
            ClawSubsystem s_Claw,
            ArmElevatorMech s_ArmElevator,
            ElevatorMech s_Elevator,
            IntakeMech s_Intake) {
        return new Place(
                nodeId, false, s_Swerve, s_Limelight, s_Claw, s_ArmElevator, s_Elevator, s_Intake);
    }

    public Place(
            int nodeId,
            boolean isCone,
            Swerve s_Swerve,
            LimelightSubsystem s_Limelight,
            ClawSubsystem s_Claw,
            ArmElevatorMech s_ArmElevator,
            ElevatorMech s_Elevator,
            IntakeMech s_Intake) {
        addRequirements(s_Swerve, s_Limelight, s_Claw, s_ArmElevator, s_Elevator, s_Intake);
        addCommands(
                new InstantCommand(() -> RobotContainer.setIsCone(isCone)),
                nodeId == 0
                        ? new MidNodePosition(s_Elevator, s_ArmElevator, s_Claw)
                        : new HighNodePosition(s_Elevator, s_ArmElevator, s_Claw),
                new WaitCommand(0.5),
                new AutonIntakeNoCurrentLimit(s_Intake, isCone).withTimeout(0.3),
                new InstantCommand(() -> s_Intake.intake(0, false)),
                new StowPosition(s_Elevator, s_ArmElevator, s_Claw).withTimeout(0.1));
    }
}
