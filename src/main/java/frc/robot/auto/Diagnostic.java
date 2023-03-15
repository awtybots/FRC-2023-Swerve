/** Thank you GOFIRST-Robotics! */
package frc.robot.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.Positions.StowPosition;
import frc.robot.subsystems.MechanicalParts.ArmSubsystem;
import frc.robot.subsystems.MechanicalParts.ClawSubsystem;
import frc.robot.subsystems.MechanicalParts.ElevatorSubsystem;
import frc.robot.subsystems.MechanicalParts.IntakeSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class Diagnostic extends CommandBase {

    private final ElevatorSubsystem s_Elevator;
    private final ArmSubsystem s_Arm;
    private final ClawSubsystem s_Claw;
    private final IntakeSubsystem s_Intake;
    private final Swerve s_Swerve;

    /** Command to use Gyro data to resist the tip angle from the beam - to stabilize and balance. */
    public Diagnostic(
            ElevatorSubsystem s_Elevator,
            ArmSubsystem s_Arm,
            ClawSubsystem s_Claw,
            IntakeSubsystem s_Intake,
            Swerve s_Swerve) {
        addRequirements(s_Elevator, s_Arm, s_Claw, s_Intake, s_Swerve);
        this.s_Elevator = s_Elevator;
        this.s_Arm = s_Arm;
        this.s_Claw = s_Claw;
        this.s_Intake = s_Intake;
        this.s_Swerve = s_Swerve;
    }

    @Override
    public void execute() {
        s_Elevator.drive(0.1);
        s_Arm.drive(0.05);
        s_Claw.driveClaw(0.025);
        s_Intake.intake(.2, true);
        s_Swerve.drive(new Translation2d(0.3, 0.3), 0, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        new StowPosition(s_Elevator, s_Arm, s_Claw);
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
