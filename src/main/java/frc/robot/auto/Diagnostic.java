/** Thank you GOFIRST-Robotics! */
package frc.robot.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Positions.StowPos;
import frc.robot.subsystems.MechanicalParts.ArmMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;
import frc.robot.subsystems.MechanicalParts.IntakeMech;
import frc.robot.subsystems.Swerve.Swerve;

public class Diagnostic extends CommandBase {

    private final ElevatorMech s_elevator;
    private final ArmMech s_arm;
    private final ClawSubsystem s_claw;
    private final IntakeMech s_intake;
    private final Swerve s_swerve;

    public Diagnostic(
            ElevatorMech s_elevator,
            ArmMech s_arm,
            ClawSubsystem s_claw,
            IntakeMech s_intake,
            Swerve s_swerve) {
        addRequirements(s_elevator, s_arm, s_claw, s_intake, s_swerve);
        this.s_elevator = s_elevator;
        this.s_arm = s_arm;
        this.s_claw = s_claw;
        this.s_intake = s_intake;
        this.s_swerve = s_swerve;
    }

    @Override
    public void execute() {
        s_elevator.drive(0.1);
        s_arm.drive(0.05);
        s_claw.driveClaw(0.025);
        s_intake.intake(.2, true);
        s_swerve.drive(new Translation2d(0.3, 0.3), 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        new StowPos(s_elevator, s_arm, s_claw);
        s_intake.intake(0, false);
        s_swerve.drive(new Translation2d(0.3, 0), 0, false);
    }
}
