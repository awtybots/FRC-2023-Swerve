/** Thank you GOFIRST-Robotics! */
package frc.robot.commands.Autonomous.Balance;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve.Swerve;

public class PreciseBalance extends CommandBase {

    private final Swerve s_swerve;

    private final double kPTheta = 1;
    private final double kReversingMultiplier = 3;
    private final double kThetaThreshold = 1;

    private double decrease = 1;

    /** Command to use Gyro data to resist the tip angle from the beam - to stabilize and balance. */
    public PreciseBalance(Swerve s_swerve) {
        addRequirements(s_swerve);
        this.s_swerve = s_swerve;
    }

    @Override
    public void execute() {
        double currentAngle = s_swerve.getRoll();
        double error = 0.0 - currentAngle;

        if (Math.abs(error) < kThetaThreshold * 5) {
            decrease += 0.07;
        }

        double drivePower = -Math.min(kPTheta * error, 3.7);

        // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
        if (drivePower < 0) {
            drivePower *= kReversingMultiplier;
        }

        // Limit the max power
        if (Math.abs(drivePower) > 0.6) {
            drivePower = Math.copySign(0.6, drivePower);
        }

        var translation = new Translation2d((drivePower / decrease), 0);
        s_swerve.drive(translation, 0, false);
    }

    @Override
    public void end(boolean interrupted) {
        s_swerve.drive(new Translation2d(), 0, true);
    }
}
