/** Thank you GOFIRST-Robotics! */
package frc.robot.commands.Autonomous.Balance;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class PreciseBalance extends CommandBase {

    private final Swerve s_Swerve;
    private final LedSubsystem s_Led;

    private Translation2d translation;
    private double rotation;

    private double error;
    private double currentAngle;
    private double drivePower;
    private double decrease = 1;

    /** Command to use Gyro data to resist the tip angle from the beam - to stabilize and balance. */
    public PreciseBalance(Swerve s_Swerve, LedSubsystem s_Led) {
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.s_Led = s_Led;
    }

    @Override
    public void execute() {
        if (Math.abs(error) < Constants.Balance.BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES * 5) {
            decrease += 0.1;
            // return;
        }

        currentAngle = s_Swerve.getRoll();

        error = Constants.Balance.BEAM_BALANCED_GOAL_DEGREES - currentAngle;

        drivePower = -Math.min(Constants.Balance.BEAM_BALANACED_DRIVE_KP * error, 3.7);

        // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
        if (drivePower < 0) {
            drivePower *= Constants.Balance.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
        }

        // Limit the max power
        if (Math.abs(drivePower) > 0.6) {
            drivePower = Math.copySign(0.6, drivePower);
        }

        translation = new Translation2d((drivePower / decrease), 0);
        s_Swerve.drive(translation, rotation, false);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        s_Led.setAnimation("VIVELAFRANCE", false);
        s_Swerve.drive(new Translation2d(0, 0), 0, true);
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
