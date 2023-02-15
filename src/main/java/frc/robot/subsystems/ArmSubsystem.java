package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.util.math.Convert;
import frc.util.math.Convert.Encoder;

import frc.robot.Constants.ModuleConstants;


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

    public double armHeight = 0;



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
        //mRightArmEncoder.setInverted(ModuleConstants.kTurningEncoderInverted);

        mLeftArmPIDController.setP(1);
        mLeftArmPIDController.setI(0);
        mLeftArmPIDController.setD(0);
        mLeftArmPIDController.setOutputRange(-1,
        1);

        mRightArmPIDController.setP(1);
        mRightArmPIDController.setI(0);
        mRightArmPIDController.setD(0);
        mRightArmPIDController.setOutputRange(-1,
        1);

        pidControllers = new SparkMaxPIDController[]{mLeftArmPIDController, mRightArmPIDController};

 

        //mLeftArmPIDController.setFeedbackDevice(mLeftArmEncoder);
        //mRightArmPIDController.setFeedbackDevice(mRightArmEncoder);

    }

    public void setRotation(int value){
        armHeight = value;
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(encoders[0].getPosition()/360);
    }

    public void drive(double pct) {
        // for (CANSparkMax motor : motors)
        //     motor.set(pct);
        mLeftArmMotor.set(pct*0.2);
        mRightArmMotor.set(pct*-0.2);
        armHeight += pct/10;
        SmartDashboard.putNumber("armHeight ", armHeight);
        SmartDashboard.putNumber("armEncoderReadout1 ", mLeftArmEncoder.getPosition());
        SmartDashboard.putNumber("armEncoderReadout2 ", -mRightArmEncoder.getPosition());



        
        //mLeftArmPIDController.setReference(armHeight, CANSparkMax.ControlType.kPosition);
        //mRightArmPIDController.setReference(armHeight, CANSparkMax.ControlType.kPosition);
        //mLeftArmPIDController.setReference(0.1, CANSparkMax.ControlType.kSmartMotion);
        //mRightArmPIDController.setReference(-0.1, CANSparkMax.ControlType.kSmartMotion);

    }

    @Override
    public void periodic() {
        SmartDashboard.putString("Arm... Angle?", getCanCoder().toString());
    }

    public void stop() {
       // for (CANSparkMax motor : motors)
            //motor.set(0);
            
   }

}