package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Positions.Intake.IntakeFromGroundPosition;
import frc.robot.RobotContainer;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.MechanicalParts.ArmElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.IntakeSubsystem;

public class PreparePickup extends SequentialCommandGroup {

    public PreparePickup(
            ClawSubsystem s_Claw,
            ArmElevatorSubsystem s_ArmElevator,
            ElevatorSubsystem s_Elevator,
            IntakeSubsystem s_Intake,
            boolean isCone) {
        // addRequirements(s_Claw, s_Arm, s_Elevator, s_Intake);
        // ! only isCone = true done, not finished cube and possible intake?
        addCommands(
                new InstantCommand(() -> RobotContainer.setIsCone(isCone)),
                new IntakeFromGroundPosition(s_Elevator, s_ArmElevator, s_Claw).withTimeout(0.5),
                new InstantCommand(() -> s_Intake.intake(RobotContainer.getIsCone() ? -1 : 1, true)));
    }
}
