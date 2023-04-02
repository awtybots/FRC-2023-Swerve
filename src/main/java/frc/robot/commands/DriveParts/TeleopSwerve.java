package frc.robot.commands.DriveParts;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DefaultConfig;
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.Swerve.Swerve;
import frc.util.Controller;

public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private Swerve s_Swerve;
    private Controller driver;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(DefaultConfig.rampRate);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(DefaultConfig.rampRate);

    /** Driver control */
    public TeleopSwerve(
            Swerve s_Swerve, Controller driver, int translationAxis, int strafeAxis, int rotationAxis) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.driver = driver;
        this.fieldRelative = DefaultConfig.fieldRelative;
    }

    @Override
    public void execute() {

        double yAxis = driver.getLeftStickY();
        double xAxis = -driver.getLeftStickX();
        double rAxis = -driver.getRightStickX();

        // Reduces speed if low speed mode is activated
        if (!s_Swerve.swerveHighSpeedMode) {
            yAxis *= s_Swerve.swerveSpeedValue;
            xAxis *= s_Swerve.swerveSpeedValue;
            rAxis *= s_Swerve.swerveSpeedValue;
        }

        translation =
                new Translation2d(yLimiter.calculate(yAxis), xLimiter.calculate(xAxis))
                        .times(Drivetrain.kMaxSpeedMetersPerSecond);
        rotation = rAxis * Drivetrain.kMaxAngularSpeed;
        s_Swerve.drive(translation, rotation, true);
    }
}
