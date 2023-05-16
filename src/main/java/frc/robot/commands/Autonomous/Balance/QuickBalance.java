/** Thank you GOFIRST-Robotics! */
package frc.robot.commands.Autonomous.Balance;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.Swerve;

public class QuickBalance extends CommandBase {

    private static final double kPTheta = 1;
    private static final double kReversingMultiplier = 3;
    private static final double kThetaThreshold = 3;

    private final Swerve s_swerve;

    private double error;

    /** Command to use Gyro data to resist the tip angle from the beam - to stabilize and balance. */
    public QuickBalance(Swerve s_swerve) {
        addRequirements(s_swerve);
        this.s_swerve = s_swerve;
    }

    @Override
    public void execute() {
        double currentAngle = s_swerve.getRoll();
        error = 0.0 - currentAngle;

        double drivePower = -Math.min(kPTheta * error, 3.7);

        // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
        if (drivePower < 0) {
            drivePower *= kReversingMultiplier;
        }

        // Limit the max power
        if (Math.abs(drivePower) > 0.4) {
            drivePower = Math.copySign(0.4, drivePower);
        }

        var translation = new Translation2d(drivePower, 0);
        s_swerve.drive(translation, 0.0, false);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(error) < kThetaThreshold;
    }
}
