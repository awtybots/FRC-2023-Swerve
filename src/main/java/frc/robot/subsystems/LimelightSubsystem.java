package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {
    
    NetworkTable table;
    double tx;
    double ty;
    double ta;

    String jsonDump;
    double ry;

    //TODO: LED | LedSubsystem s_LEDSubsystem;

    //TODO: LED | public LimelightSubsystem(LedSubsystem s_LEDSubsystem){
    public LimelightSubsystem(){
        //TODO: LED | this.s_LEDSubsystem = s_LEDSubsystem;
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public double horizontalOffset(){
        return tx;
    }   
    public double horizontalRotation(){
        return ry;
    }

    public void setMode(int number) {
        table.getEntry("ledMode").setDouble(number);
    }

    @Override
    public void periodic() {
        tx = table.getEntry("tx").getDouble(0.0);
        ty = table.getEntry("ty").getDouble(0.0);
        ta = table.getEntry("ta").getDouble(0.0);

        // TODO: 3D ? (experimental)
        jsonDump = NetworkTableInstance.getDefault().getTable("limelight").getEntry("json").getString("");
        try {
            jsonDump
        }
        // ry = targetpose_cameraspace[1];


        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);

        SmartDashboard.putString("JsonDump", jsonDump);
        // for(int i=0; i<targetpose_cameraspace.length; i++) {
        //     SmartDashboard.putNumber("targetpose_cameraspace" + i, targetpose_cameraspace[i]);
        // }
        SmartDashboard.putNumber("LimeLightRY", ry);

        //TODO: LED | s_LEDSubsystem.visionTrackingLED(area);
    }
}