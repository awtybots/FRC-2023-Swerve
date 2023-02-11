package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.util.math.Convert;
import frc.util.math.Convert.Encoder;

public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax mLeftArmMotor;
    private CANSparkMax mRightArmMotor;

    private final RelativeEncoder mLeftArmEncoder;
    private final RelativeEncoder mRightArmEncoder;

    private final SparkMaxPIDController mLeftArmPIDController;
    private final SparkMaxPIDController mRightArmPIDController;

    private final CANSparkMax[] motors;
    private final RelativeEncoder[] encoders;
    private final SparkMaxPIDController[] pidControllers;

    public ArmSubsystem() {
        mLeftArmMotor = new CANSparkMax(Constants.ArmConstants.kRightArmMotorId, MotorType.kBrushless);
        mRightArmMotor = new CANSparkMax(Constants.ArmConstants.kLeftArmMotorId, MotorType.kBrushless);
        mLeftArmMotor.restoreFactoryDefaults();
        mRightArmMotor.restoreFactoryDefaults();
        motors = new CANSparkMax[] {mLeftArmMotor, mRightArmMotor};

        mLeftArmEncoder = mLeftArmMotor.getEncoder();
        mRightArmEncoder = mRightArmMotor.getEncoder();
        encoders = new RelativeEncoder[] {mLeftArmEncoder, mRightArmEncoder};

        mLeftArmPIDController = mRightArmMotor.getPIDController();
        mRightArmPIDController = mRightArmMotor.getPIDController();
        pidControllers = new SparkMaxPIDController[]{mLeftArmPIDController, mRightArmPIDController};

 

        //mLeftArmPIDController.setFeedbackDevice(mLeftArmEncoder);
        //mRightArmPIDController.setFeedbackDevice(mRightArmEncoder);

    }

    public void drive(double pct) {
        // for (CANSparkMax motor : motors)
        //     motor.set(pct);
        //mLeftArmMotor.set(pct*0.3);
        //mRightArmMotor.set(-pct*0.3);
        
        mLeftArmPIDController.setReference(pct*0.6, CANSparkMax.ControlType.kSmartMotion);
        mRightArmPIDController.setReference(-pct*0.6, CANSparkMax.ControlType.kSmartMotion);

    }

    public void stop() {
        for (CANSparkMax motor : motors)
            motor.set(0);
    }

}