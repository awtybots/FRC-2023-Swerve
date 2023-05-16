package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.Swerve.Swerve;

public class VisionTracking extends CommandBase {

    private final Swerve s_Swerve;
    private final LimelightSubsystem s_Limelight;

    private double Beta;

    private final double rotateThreshold = 2;
    private final double rotateSpeed = 0.5;
    private final double offset = 7.5;

    public VisionTracking(Swerve s_Swerve, LimelightSubsystem s_Limelight) {
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.s_Limelight = s_Limelight;
    }

    @Override
    public void execute() {
        if (!s_Limelight.hasTarget()) return;

        double Beta = s_Limelight.getHorizontalOffset();

        double rotation = 0;
        if (Math.abs(Beta - offset) > rotateThreshold) {
            rotation = -Math.signum(Beta) * rotateSpeed;
        }

        s_Swerve.drive(new Translation2d(), rotation, true);
    }

    @Override
    public boolean isFinished() {
        return !s_Limelight.hasTarget() || Math.abs(Beta - offset) < rotateThreshold;
    }
}
