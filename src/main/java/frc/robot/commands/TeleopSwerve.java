package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Swerve;
import frc.util.Controller;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    
    private Swerve s_Swerve;
    private Controller controller;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(Constants.CustomConstants.rampRate);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(Constants.CustomConstants.rampRate);

    private double applyDeadband(double axisValue, double deadbandValue) {
        return (Math.abs(axisValue) < deadbandValue ? 0 : axisValue);
    }


    /**
     * Driver control
     */
    public TeleopSwerve(Swerve s_Swerve, Controller controller) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.fieldRelative = Constants.CustomConstants.fieldRelative;
    }

    @Override
    public void execute() {
        if(RobotContainer.isAutoTargetOn) return;

        double yAxis = controller.getLeftStickY();
        double xAxis = -controller.getLeftStickX();
        double rAxis = -controller.getRightStickX();
        
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
        s_Swerve.drive(translation, rotation, fieldRelative);
    }
}