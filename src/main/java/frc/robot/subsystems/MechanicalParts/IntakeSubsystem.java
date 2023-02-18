package frc.robot.subsystems.MechanicalParts;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax mLeftIntakeMotor;
    private CANSparkMax mRightIntakeMotor;

    private final RelativeEncoder mLeftIntakeEncoder;
    private final RelativeEncoder mRightIntakeEncoder;

    // private final SparkMaxPIDController mLeftIntakePIDController;
    // private final SparkMaxPIDController mRightIntakePIDController;

    private final CANSparkMax[] intakeMotors;
    // private final RelativeEncoder[] intakeEncoders;
    // private final SparkMaxPIDController[] pidControllers;

    public IntakeSubsystem() {
        mLeftIntakeMotor =
                new CANSparkMax(Constants.ClawConstants.kLeftIntakeMotorId, MotorType.kBrushless);
        mRightIntakeMotor =
                new CANSparkMax(Constants.ClawConstants.kRightIntakeMotorId, MotorType.kBrushless);

        mLeftIntakeMotor.restoreFactoryDefaults();

        mRightIntakeMotor.restoreFactoryDefaults();

        intakeMotors = new CANSparkMax[] {mLeftIntakeMotor, mRightIntakeMotor};

        mLeftIntakeEncoder = mLeftIntakeMotor.getEncoder();
        mRightIntakeEncoder = mRightIntakeMotor.getEncoder();
        // intakeEncoders = new RelativeEncoder[] {mLeftIntakeEncoder, mRightIntakeEncoder};

        // mLeftIntakePIDController = mLeftIntakeMotor.getPIDController();
        // mRightIntakePIDController = mRightIntakeMotor.getPIDController();
        // pidControllers = new SparkMaxPIDController[]{mLeftIntakePIDController,
        // mRightIntakePIDController};

        // mLeftIntakePIDController.setFeedbackDevice(mLeftIntakeEncoder);
        // mRightIntakePIDController.setFeedbackDevice(mRightIntakeEncoder);

    }

    public void intake(double pct) {

        double multiplier;
        if (pct > 0) {
            multiplier = 1;
        } else {
            multiplier = 1;
        }
        intakeMotors[0].set(-pct * multiplier);
        intakeMotors[1].set(pct * multiplier);
    }

    public void stopIntake() {
        for (CANSparkMax motor : intakeMotors) motor.set(0);
    }
}
