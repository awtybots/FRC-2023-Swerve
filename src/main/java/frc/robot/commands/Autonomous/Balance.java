/** Thank you GOFIRST-Robotics! */
package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve.Swerve;

public class Balance extends CommandBase {

    private final Swerve s_Swerve;

    private Translation2d translation;
    private double rotation;

    private double error;
    private double currentAngle;
    private double drivePower;

    /** Command to use Gyro data to resist the tip angle from the beam - to stabilize and balance. */
    public Balance(Swerve s_Swerve) {
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
    }

    @Override
    public void execute() {

        this.currentAngle = s_Swerve.getPitch();

        error = Constants.Balance.BEAM_BALANCED_GOAL_DEGREES - currentAngle;
        drivePower = -Math.min(Constants.Balance.BEAM_BALANACED_DRIVE_KP * error, 1);

        // Our robot needed an extra push to drive up in reverse, probably due to weight imbalances
        if (drivePower < 0) {
            drivePower *= Constants.Balance.BACKWARDS_BALANCING_EXTRA_POWER_MULTIPLIER;
        }

        // Limit the max power
        if (Math.abs(drivePower) > 0.4) {
            drivePower = Math.copySign(0.4, drivePower);
        }

        translation = new Translation2d(drivePower, 0);
        s_Swerve.drive(translation, rotation, true);

        // Debugging Print Statments
        System.out.println("Current Angle: " + currentAngle);
        System.out.println("Error " + error);
        System.out.println("Drive Power: " + drivePower);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(error)
                < Constants.Balance
                        .BEAM_BALANCED_ANGLE_TRESHOLD_DEGREES; // End the command when we are within the
        // specified threshold of being 'flat' (gyroscope
        // pitch of 0 degrees)
    }
}
