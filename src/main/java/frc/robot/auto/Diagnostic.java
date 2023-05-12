/** Thank you GOFIRST-Robotics! */
package frc.robot.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.MechanicalParts.ArmElevatorMech;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorMech;
import frc.robot.subsystems.MechanicalParts.IntakeMech;
import frc.robot.subsystems.Swerve.Swerve;

public class Diagnostic extends CommandBase {

    private final ElevatorMech s_Elevator;
    private final ArmElevatorMech s_ArmElevator;
    private final ClawSubsystem s_Claw;
    private final IntakeMech s_Intake;
    private final Swerve s_Swerve;

    /** Command to use Gyro data to resist the tip angle from the beam - to stabilize and balance. */
    public Diagnostic(
            ElevatorMech s_Elevator,
            ArmElevatorMech s_ArmElevator,
            ClawSubsystem s_Claw,
            IntakeMech s_Intake,
            Swerve s_Swerve) {
        addRequirements(s_Elevator, s_ArmElevator, s_Claw, s_Intake, s_Swerve);
        this.s_Elevator = s_Elevator;
        this.s_ArmElevator = s_ArmElevator;
        this.s_Claw = s_Claw;
        this.s_Intake = s_Intake;
        this.s_Swerve = s_Swerve;
    }

    @Override
    public void execute() {
        s_Elevator.drive(0.1);
        s_ArmElevator.drive(0.05);
        s_Claw.driveClaw(0.025);
        s_Intake.intake(.2, true);
        s_Swerve.drive(new Translation2d(0.3, 0.3), 0, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        new StowPosition(s_Elevator, s_ArmElevator, s_Claw);
        s_Intake.intake(0, false);
        s_Swerve.drive(new Translation2d(0.3, 0), 0, false);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
