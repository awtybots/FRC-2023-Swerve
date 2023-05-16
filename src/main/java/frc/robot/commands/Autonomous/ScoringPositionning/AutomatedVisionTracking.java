package frc.robot.commands.Autonomous.ScoringPositionning;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class AutomatedVisionTracking extends CommandBase {

    private final Swerve s_Swerve;
    private final LimelightSubsystem s_Limelight;

    private double Beta;

    private final double rotateSpeed = 0.5;
    private final double rotateThreshold = 2;

    public AutomatedVisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight) {
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;

        this.Beta = Math.toRadians(s_Limelight.getHorizontalOffset());
    }

    private void visionTracking() {
        var translation = new Translation2d(0, 0);

        Beta = Math.toRadians(s_Limelight.getHorizontalOffset());
        double rotation = -Math.signum(Beta) * rotateSpeed;
        s_Swerve.drive(translation, rotation, false);
    }

    @Override
    public void execute() {
        visionTracking();
    }

    @Override
    public void end(boolean interrupted) {
        s_Swerve.drive(new Translation2d(0, 0), 0, true);

        // reverts to the two reflective tapes mode if it's set to only detect one
        if (s_Limelight.getPipeline() == 2) {
            s_Limelight.setPipeline(1);
        }
    }

    @Override
    public boolean isFinished() {
        return !s_Limelight.hasTarget() || Math.abs(Math.toDegrees(Beta)) < rotateThreshold;
    }
}
