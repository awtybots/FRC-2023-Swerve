package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.LimelightHelpers;
// import com.kauailabs.navx.frc.AHRS;
// import edu.wpi.first.wpilibj.SPI;

// import frc.robot.SwerveModule;
// import frc.robot.Constants;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.SwerveModuleState;

public class LimelightSubsystem extends SubsystemBase {

    NetworkTable table;
    double tx;
    double ty;
    double ta;

    double[] targetPose_CameraSpace;
    double ry;
    double distance;

    // TODO: LED | LedSubsystem s_LEDSubsystem;

    // TODO: LED | public LimelightSubsystem(LedSubsystem s_LEDSubsystem){
    public LimelightSubsystem() {
        // TODO: LED | this.s_LEDSubsystem = s_LEDSubsystem;
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double getHorizontalOffset() {
        return tx;
    }

    public double getHorizontalRotation() {
        return ry;
    }

    public double getArea() {
        return ta;
    }

    public void setMode(int number) {
        table.getEntry("ledMode").setDouble(number);
    }

    public void setPipeline(int number) {
        if (number > 1) return;
        table.getEntry("pipeline").setInteger(number);
    }

    public long getPipeline() {
        return table.getEntry("pipeline").getInteger(getPipeline());
    }

    // Reflective

    @Override
    public void periodic() {
        // Reflective

        //     double[] leftData = table.getEntry("left").getDoubleArray();
        // double[] rightData = table.getEntry("right").getDoubleArray();

        tx = LimelightHelpers.getTX("");
        ty = LimelightHelpers.getTY("");
        ta = LimelightHelpers.getTA("");

        // TODO: 3D ? (experimental)
        targetPose_CameraSpace = LimelightHelpers.getTargetPose_CameraSpace("");
        ry = targetPose_CameraSpace[4];

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);

        for (int i = 0; i < targetPose_CameraSpace.length; i++) {
            SmartDashboard.putNumber("targetpose_cameraspace" + i, targetPose_CameraSpace[i]);
        }
        SmartDashboard.putNumber("LimeLightRY", ry);

        // TODO: LED | s_LEDSubsystem.visionTrackingLED(area);
    }
}
