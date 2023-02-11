package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {

    private CANSparkMax mPivotMotor;

    // private final RelativeEncoder mPivotEncoder;
    
    // private final SparkMaxPIDController mPivotPIDController;

    public ClawSubsystem() {
        mPivotMotor = new CANSparkMax(Constants.ClawConstants.kPivotMotorId, MotorType.kBrushless);
        mPivotMotor.restoreFactoryDefaults();

        // mPivotEncoder = mPivotMotor.getEncoder();
        
        // mPivotPIDController = mPivotMotor.getPIDController();
    
        // mPivotPIDController.setFeedbackDevice(mPivotEncoder);
    }

    public void driveClaw(double pct) {
        mPivotMotor.set(pct);
    }

    public void stopClaw() {
        mPivotMotor.set(0);
    }
}