package frc.robot.commands.Autonomous.ScoringPositionning;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class AutomatedVisionTracking extends CommandBase {

    private final Swerve s_Swerve;
    private final LimelightSubsystem s_Limelight;

    private double aprilBeta;
    private double aprilAlpha;
    private double reflectiveBeta;
    private double reflectiveAlpha;

    private final double kRotateThreshold = 2; // degrees
    private final double kDriveThreshold = 3; // degrees
    private final double kRotateSpeed = 0.5;
    private final double kReflectiveThreshold = 3; // degrees
    private final double kDriveSpeed = 0.5;
    private final double kReflectiveOffset = 84; // degrees
    private final double kAprilOffset = 7.5; // degrees

    public AutomatedVisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight) {
        this(s_Swerve, s_Limelight, false);
    }

    public AutomatedVisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight, boolean isCone) {
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;

        s_Limelight.setPipeline(isCone ? 1 : 0);
    }

    @Override
    public void initialize() {
        this.aprilBeta = s_Limelight.getHorizontalOffset();
        this.aprilAlpha = s_Limelight.getHorizontalRotation();
        this.reflectiveBeta = s_Limelight.getHorizontalOffset();
        this.reflectiveAlpha = s_Limelight.getSkew();
    }

    private double getSign(double num) {
        return Math.signum(num);
    }

    private boolean withinThreshold(double value, double threshold) {
        return Math.abs(value) < threshold;
    }

    private void AprilTagLogic() {
        if (!s_Limelight.hasTarget()) return;

        double rotation = 0;
        var translation = new Translation2d();

        // Rotate until beta is right
        if (withinThreshold(aprilBeta - kAprilOffset, kRotateThreshold)) {
            rotation = -getSign(aprilBeta) * kRotateSpeed;
        }

        // Strafe until Alpha is right
        if (withinThreshold(aprilAlpha, kDriveThreshold)) {
            translation = new Translation2d(0, getSign(aprilAlpha) * kDriveSpeed);
        }

        s_Swerve.drive(translation, rotation, false);
    }

    private void ReflectiveTapeLogic() {
        // Sets pipeline to number 2 (one reflective tape) if there is no target
        if (!s_Limelight.hasTarget() && s_Limelight.getPipeline() == 1) {
            s_Limelight.setPipeline(2);
        }

        double rotation = 0;
        var translation = new Translation2d();

        if (s_Limelight.hasTarget() && s_Limelight.getPipeline() == 1) {
            // Strafe until Alpha is right
            if (withinThreshold(reflectiveBeta - kReflectiveOffset, kReflectiveThreshold)) {
                translation = new Translation2d(0, -getSign(reflectiveBeta - 45) * kDriveSpeed);
            }

            // Rotate until beta is right
            if (withinThreshold(reflectiveBeta - kAprilOffset, kReflectiveThreshold)) {
                rotation = -getSign(reflectiveBeta) * kRotateSpeed;
            }
        }

        s_Swerve.drive(translation, rotation, false);
    }

    @Override
    public void execute() {
        int pipelineId = s_Limelight.getPipeline();
        if (pipelineId == 0) {
            this.aprilBeta = s_Limelight.getHorizontalOffset();
            this.aprilAlpha = s_Limelight.getHorizontalRotation();
            AprilTagLogic();
        } else {
            this.reflectiveBeta = s_Limelight.getHorizontalOffset();
            this.reflectiveAlpha = s_Limelight.getSkew();
            ReflectiveTapeLogic();
        }
    }

    @Override
    public void end(boolean interrupted) {
        // reverts to the two reflective tapes mode if it's set to only detect one
        if (s_Limelight.getPipeline() == 2) {
            s_Limelight.setPipeline(1);
        }
    }

    @Override
    public boolean isFinished() {
        if (s_Limelight.getPipeline() == 0) {
            return !s_Limelight.hasTarget()
                    || withinThreshold(aprilBeta - kAprilOffset, kRotateThreshold)
                            && withinThreshold(aprilAlpha, kDriveThreshold);
        } else if (s_Limelight.getPipeline() == 1) {
            return withinThreshold(reflectiveBeta - kAprilOffset, kReflectiveThreshold)
                    && withinThreshold(reflectiveAlpha - kReflectiveOffset, kReflectiveThreshold);
        } else {
            return !s_Limelight.hasTarget()
                    || withinThreshold(aprilBeta - kAprilOffset, kRotateThreshold);
        }
    }
}
