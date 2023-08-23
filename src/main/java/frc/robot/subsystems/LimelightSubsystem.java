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
    boolean tv;

    double ts;

    double ltx;
    double lty;
    double utx;
    double uty;

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

    public double getLowerHorizontalOffset() {
        return ltx;
    }

    public double getLowerVerticalOffset() {
        return lty;
    }

    public double getUpperHorizontalOffset() {
        return utx;
    }

    public double getUpperVerticalOffset() {
        return uty;
    }

    public double getHorizontalRotation() {
        return ry;
    }

    public double getSkew() {
        return ts;
    }

    public double getArea() {
        return ta;
    }

    public boolean hasTarget() {
        return tv;
    }

    public void setMode(int number) {
        table.getEntry("ledMode").setDouble(number);
    }

    public void setPipeline(int number) {
        table.getEntry("pipeline").setDouble(number);
    }

    public long getPipeline() {
        return table.getEntry("pipeline").getInteger(0);
    }

    // Reflective

    @Override
    public void periodic() {
        // Reflective
        tx = LimelightHelpers.getTX("");
        ty = LimelightHelpers.getTY("");
        ta = LimelightHelpers.getTA("");
        ts = table.getEntry("ts").getDouble(0);
        tv = LimelightHelpers.getTV("");

        ltx = table.getEntry("tx0").getDouble(0);
        lty = table.getEntry("ty0").getDouble(0);
        utx = table.getEntry("tx1").getDouble(0);
        uty = table.getEntry("ty1").getDouble(0);

        // TODO: 3D ? (experimental)
        targetPose_CameraSpace = LimelightHelpers.getTargetPose_CameraSpace("");
        // ry = targetPose_CameraSpace[4];
        ry = 0;

        // post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightSkew", ts);
        SmartDashboard.putNumber("LimelightArea", ta);
        SmartDashboard.putNumber("LimelightRY", ry);

        SmartDashboard.putNumber("Get Pipeline", getPipeline());

        // TODO: LED | s_LEDSubsystem.visionTrackingLED(area);
    }
}
