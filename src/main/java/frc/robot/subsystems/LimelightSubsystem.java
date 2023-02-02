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
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;



public class LimelightSubsystem extends SubsystemBase {
    
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;

    NetworkTableEntry ry;

    double x;
    double y;
    double area;

    double rotationY;


    AddressableLED m_led;
    AddressableLEDBuffer m_ledBuffer;

    public LimelightSubsystem(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        m_led = new AddressableLED(9);
        m_ledBuffer = new AddressableLEDBuffer(120);
        m_led.setLength(m_ledBuffer.getLength());

         // Set the data for led strips
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public double horizontalOffset(){
        return x;
    }   

    public void setMode(int number) {
        table.getEntry("ledMode").setDouble(number);
    }

    @Override
    public void periodic() {
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");

        ry = table.getEntry("ry");

        //read values periodically
        //horizontal offset
        x = tx.getDouble(0.0);
        //vertical offset
        y = ty.getDouble(0.0);
        //target area
        area = ta.getDouble(0.0);

        rotationY = ry.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

        SmartDashboard.putNumber("LimeLightRY", rotationY);

        for (var i = 0; i < m_ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            if (area > 17) {
                m_ledBuffer.setRGB(i, 0, 200, 0);
            }
            
            else if (area*10 > i){
                m_ledBuffer.setRGB(i, 0, 0, 200);
            }
            else{
                m_ledBuffer.setRGB(i, 100, 100, 100);
            }
         }
         
         m_led.setData(m_ledBuffer);
        }
        


}