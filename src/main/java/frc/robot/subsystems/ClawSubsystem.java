package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {

    private CANSparkMax mPivotMotor;



    private final RelativeEncoder mPivotEncoder;
    
    private final SparkMaxPIDController mPivotPIDController;
    public double wristHeight;

    public ClawSubsystem() {
        mPivotMotor = new CANSparkMax(Constants.ClawConstants.kPivotMotorId, MotorType.kBrushless);
        mPivotMotor.restoreFactoryDefaults();

        mPivotPIDController =  mPivotMotor.getPIDController();

        mPivotEncoder = mPivotMotor.getEncoder();
        
        mPivotPIDController.setP(0.4);
        mPivotPIDController.setI(0.1);
        mPivotPIDController.setD(0.15);
        mPivotPIDController.setOutputRange(-0.2,
        0.2);
    
        // mPivotPIDController.setFeedbackDevice(mPivotEncoder);
    }

    public void driveClaw(double pct) {
        
        SmartDashboard.putNumber("wristHeight ", wristHeight);
        SmartDashboard.putNumber("wristEncoderReadout1 ", mPivotEncoder.getPosition());
        mPivotPIDController.setReference(wristHeight, CANSparkMax.ControlType.kPosition);
        wristHeight += pct/3.0;

    }


    public void stopClaw() {
        mPivotMotor.set(0);
    }
}