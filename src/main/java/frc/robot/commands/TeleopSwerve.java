package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.util.Controller;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private Swerve s_Swerve;
    private Controller driver;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    private SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.CustomConstants.rampRate);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.CustomConstants.rampRate);

    private double applyDeadband(double axisValue, double deadbandValue) {
        return (Math.abs(axisValue) < deadbandValue ? 0 : axisValue);
    }


    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Controller driver, int translationAxis, int strafeAxis, int rotationAxis) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.driver = driver;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = Constants.CustomConstants.fieldRelative;
    }

    @Override
    public void execute() {
        double yAxis = -driver.getRawAxis(translationAxis);
        double xAxis = -driver.getRawAxis(strafeAxis);
        double rAxis = -driver.getRawAxis(rotationAxis);
        
        yAxis = applyDeadband(yAxis, Constants.CustomConstants.stickDeadband);
        xAxis = applyDeadband(xAxis, Constants.CustomConstants.stickDeadband);
        rAxis = applyDeadband(rAxis, Constants.CustomConstants.stickDeadband);

        // Reduces speed if low speed mode is activated
        if(!s_Swerve.swerveHighSpeedMode){
            yAxis *= Constants.CustomConstants.lowSpeedMultiplier;
            xAxis *= Constants.CustomConstants.lowSpeedMultiplier;
            rAxis *= Constants.CustomConstants.lowSpeedMultiplier;
        }

        translation = new Translation2d(yLimiter.calculate(yAxis), xLimiter.calculate(xAxis)).times(Constants.DriveConstants.kMaxSpeedMetersPerSecond);
        rotation = rAxis * Constants.DriveConstants.kMaxAngularSpeed;
        s_Swerve.drive(translation, rotation, true);
    }
}