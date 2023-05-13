package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Positions.Intake.IntakeGroundPos;
import frc.robot.subsystems.MechanicalParts.ArmElevatorMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;
import frc.robot.subsystems.MechanicalParts.IntakeMech;

public class PreparePickup extends SequentialCommandGroup {

    public PreparePickup(
            boolean isCone,
            ClawSubsystem s_Claw,
            ArmElevatorMech s_ArmElevator,
            ElevatorMech s_Elevator,
            IntakeMech s_Intake) {
        // addRequirements(s_Claw, s_Arm, s_Elevator, s_Intake);
        // ! only isCone = true done, not finished cube and possible intake?
        addCommands(
                new InstantCommand(() -> RobotContainer.enableConeMode(isCone)),
                new IntakeGroundPos(s_Elevator, s_ArmElevator, s_Claw).withTimeout(0.5),
                new InstantCommand(
                        () -> s_Intake.intake(RobotContainer.coneModeEnabled() ? -0.9 : 0.9, true)));
    }
}
