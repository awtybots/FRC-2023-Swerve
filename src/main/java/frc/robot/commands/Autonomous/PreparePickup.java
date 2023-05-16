package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Positions.Intake.IntakeGroundPos;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;
import frc.robot.subsystems.MechanicalParts.IntakeMech;

public class PreparePickup extends SequentialCommandGroup {

    public static PreparePickup Cone(
            ClawSubsystem s_claw, ArmMech s_arm, ElevatorMech s_elevator, IntakeMech s_intake) {
        return new PreparePickup(true, s_claw, s_arm, s_elevator, s_intake);
    }

    public static PreparePickup Cube(
            ClawSubsystem s_claw, ArmMech s_arm, ElevatorMech s_elevator, IntakeMech s_intake) {
        return new PreparePickup(false, s_claw, s_arm, s_elevator, s_intake);
    }

    private PreparePickup(
            boolean isCone,
            ClawSubsystem s_Claw,
            ArmMech s_Arm,
            ElevatorMech s_Elevator,
            IntakeMech s_Intake) {
        // addRequirements(s_Claw, s_Arm, s_Elevator, s_Intake);
        addCommands(
                new InstantCommand(() -> RobotContainer.enableConeMode(isCone)),
                new IntakeGroundPos(s_Elevator, s_Arm, s_Claw).withTimeout(0.5),
                new InstantCommand(
                        () -> s_Intake.intake(RobotContainer.coneModeEnabled() ? -0.9 : 0.9, true)));
    }
}
