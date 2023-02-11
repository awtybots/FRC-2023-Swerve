package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax mLeftIntakeMotor;
    private CANSparkMax mRightIntakeMotor;

    // private final RelativeEncoder mLeftIntakeEncoder;
    // private final RelativeEncoder mRightIntakeEncoder;
    
    // private final SparkMaxPIDController mLeftIntakePIDController;
    // private final SparkMaxPIDController mRightIntakePIDController;

    private final CANSparkMax[] intakeMotors;
    // private final RelativeEncoder[] intakeEncoders;
    // private final SparkMaxPIDController[] pidControllers;

    public IntakeSubsystem() {
        mLeftIntakeMotor = new CANSparkMax(Constants.ClawConstants.kLeftIntakeMotorId, MotorType.kBrushless);
        mRightIntakeMotor = new CANSparkMax(Constants.ClawConstants.kRightIntakeMotorId, MotorType.kBrushless);

        mLeftIntakeMotor.restoreFactoryDefaults();
        mRightIntakeMotor.restoreFactoryDefaults();

        intakeMotors = new CANSparkMax[] {mLeftIntakeMotor, mRightIntakeMotor};

        // mLeftIntakeEncoder = mLeftIntakeMotor.getEncoder();
        // mRightIntakeEncoder = mRightIntakeMotor.getEncoder();
        // intakeEncoders = new RelativeEncoder[] {mLeftIntakeEncoder, mRightIntakeEncoder};
        
        // mLeftIntakePIDController = mLeftIntakeMotor.getPIDController();
        // mRightIntakePIDController = mRightIntakeMotor.getPIDController();
        // pidControllers = new SparkMaxPIDController[]{mLeftIntakePIDController, mRightIntakePIDController};

        // mLeftIntakePIDController.setFeedbackDevice(mLeftIntakeEncoder);
        // mRightIntakePIDController.setFeedbackDevice(mRightIntakeEncoder);

    }

    public void intake(double pct) {
        for (CANSparkMax motor : intakeMotors)
            motor.set(pct);
    }

    public void stopIntake() {
        for (CANSparkMax motor : intakeMotors)
            motor.set(0);
    }

}