package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Positions.Intake.IntakeGroundPos;
import frc.robot.commands.Positions.StowPos;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;
import frc.robot.subsystems.MechanicalParts.IntakeMech;

public class Pickup extends SequentialCommandGroup {

    public static Pickup Cone(
            ClawSubsystem s_claw, ArmMech s_arm, ElevatorMech s_elevator, IntakeMech s_intake) {
        return new Pickup(true, s_claw, s_arm, s_elevator, s_intake);
    }

    public static Pickup Cube(
            ClawSubsystem s_claw, ArmMech s_arm, ElevatorMech s_elevator, IntakeMech s_intake) {
        return new Pickup(false, s_claw, s_arm, s_elevator, s_intake);
    }

    private Pickup(
            boolean isCone,
            ClawSubsystem s_claw,
            ArmMech s_arm,
            ElevatorMech s_elevator,
            IntakeMech s_intake) {
        addRequirements(s_claw, s_arm, s_elevator, s_intake);
        addCommands(
                new InstantCommand(() -> RobotContainer.enableConeMode(isCone)),
                new IntakeGroundPos(s_elevator, s_arm, s_claw),
                new AutonIntakeCurrentLimit(s_intake).withTimeout(0.2),
                new StowPos(s_elevator, s_arm, s_claw));
    }
}
