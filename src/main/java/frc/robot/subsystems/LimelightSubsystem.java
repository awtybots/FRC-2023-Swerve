package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Camera;
import frc.robot.Constants.Field;
import frc.lib.util.math.Vector2;
import frc.lib.util.vision.Limelight;

public class LimelightSubsystem extends SubsystemBase {

    private final Limelight limelight;
    private final VisionTarget upperHub;

    private final int filterSize = 10;
    private final double kAngleLatencyGain = 0.5;

    private final MedianFilter distFilter = new MedianFilter(filterSize);
    private final MedianFilter thetaFilter = new MedianFilter(filterSize);
    private double angleToTarget = 0;
    private double distToTarget = 0;

    private boolean hasTarget = false;
    private final Debouncer visibilityFilter =
            new Debouncer(filterSize * 0.02, DebounceType.kFalling);

    public LimelightSubsystem() {
        upperHub = new VisionTarget(Field.kVisionTargetHeight, Field.kGoalHeight);
        limelight =
                new RotatableLimelight(
                        Camera.kMountingHeight, Camera.kMountingAngle, Camera.kShooterOffset);

        enableDrivingMode();
    }

    @Override
    public void periodic() {
        boolean hasTarget = hasVisibleTarget();
        Vector2 targetDisplacement = limelight.getDisplacementFrom(upperHub);

        if (hasTarget && targetDisplacement != null) {
            angleToTarget = thetaFilter.calculate(limelight.targetXOffset());
            distToTarget = distFilter.calculate(targetDisplacement.x + Field.kGoalRadius);
        } else {
            angleToTarget = 0;
            distToTarget = -1;
        }

        SmartDashboard.putBoolean("Target Visible", hasTarget);
        SmartDashboard.putNumber("Camera Angle Delta", angleToTarget);
        SmartDashboard.putNumber("Distance to Target", distToTarget);
    }

    public boolean hasVisibleTarget() {
        boolean hasTargetRaw = limelight.hasVisibleTarget();
        hasTarget = visibilityFilter.calculate(hasTargetRaw);
        return hasTarget;
    }

    public double distToTarget() {
        return distToTarget;
    }

    public double angleToTarget() {
        return angleToTarget * kAngleLatencyGain;
    }

    public void enableDrivingMode() {
        limelight.setPipeline(Camera.kPipelineDriving);
    }

    public void enableShootingMode() {
        limelight.setPipeline(Camera.kPipelineShooting);
    }

    public void setPipeline(int pipeline) {
        limelight.setPipeline(pipeline);
    }

    private class RotatableLimelight extends frc.util.vision.Limelight {

        public RotatableLimelight(
                double mountingHeight, double mountingAngle, Vector2 mechanismOffset) {
            super(mountingHeight, mountingAngle, mechanismOffset);
        }

        @Override
        public double targetXOffset() {
            switch (Camera.kOrientation) {
                case kLandscape:
                    return super.targetXOffset();
                case kUpsideDown:
                    return -super.targetXOffset();
                case kPortrait:
                    return super.targetYOffset();
                default:
                    return 0.0;
            }
        }

        @Override
        public double targetYOffset() {
            switch (Camera.kOrientation) {
                case kLandscape:
                    return super.targetYOffset();
                case kUpsideDown:
                    return -super.targetYOffset();
                case kPortrait:
                    return -super.targetXOffset();
                default:
                    return 0.0;
            }
        }
    }
}