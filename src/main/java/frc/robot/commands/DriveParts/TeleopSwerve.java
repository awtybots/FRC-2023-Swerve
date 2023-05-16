package frc.robot.commands.DriveParts;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DefaultConfig;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.Swerve.Swerve;
import frc.util.Controller;

public class TeleopSwerve extends CommandBase {

    private Swerve s_swerve;
    private Controller driver;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(DefaultConfig.rampRate);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(DefaultConfig.rampRate);

    public TeleopSwerve(
            Swerve s_swerve, Controller driver, int translationAxis, int strafeAxis, int rotationAxis) {
        this.s_swerve = s_swerve;
        addRequirements(s_swerve);

        this.driver = driver;
    }

    @Override
    public void execute() {

        double yAxis = driver.getLeftStickY();
        double xAxis = -driver.getLeftStickX();
        double rAxis = -driver.getRightStickX();

        // Reduces speed if low speed mode is activated
        if (!s_swerve.swerveHighSpeedMode) {
            yAxis *= 0.2;
            xAxis *= 0.2;
            rAxis *= 0.2;
        }

        var translation =
                new Translation2d(yLimiter.calculate(yAxis), xLimiter.calculate(xAxis))
                        .times(Drivetrain.kMaxSpeedMetersPerSecond);
        double rotation = rAxis * Drivetrain.kMaxAngularSpeed;
        s_swerve.drive(translation, rotation, true);
    }
}
