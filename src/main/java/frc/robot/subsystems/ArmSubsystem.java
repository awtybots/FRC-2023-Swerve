package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;

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


public class ArmSubsystem extends SubsystemBase {

    private CANSparkMax mLeftArmMotor;
    private CANSparkMax mRightArmMotor;

    private final RelativeEncoder mRightArmEncoder;

    private final SparkMaxPIDController mRightArmPIDController;

    public double armHeight = 0;



    public ArmSubsystem() {
        mLeftArmMotor = new CANSparkMax(Constants.ArmConstants.kRightArmMotorId, MotorType.kBrushless);
        mRightArmMotor = new CANSparkMax(Constants.ArmConstants.kLeftArmMotorId, MotorType.kBrushless);
        // mLeftArmMotor.restoreFactoryDefaults();
        mRightArmMotor.restoreFactoryDefaults();
        mRightArmMotor.setInverted(true);
        mLeftArmMotor.follow(mRightArmMotor, true);

        mRightArmEncoder = mRightArmMotor.getEncoder();

        mRightArmPIDController = mRightArmMotor.getPIDController();;

        mRightArmPIDController.setP(0.04);
        mRightArmPIDController.setI(0);
        mRightArmPIDController.setD(0);
        mRightArmPIDController.setOutputRange(-1,
        1);


        //mRightArmPIDController.setFeedbackDevice(mRightArmEncoder);

    }

    public void setRotation(int value){
        armHeight = value;
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(mRightArmEncoder.getPosition()/360);

    }

    public void drive(double pct) {
        // mRightArmMotor.set(pct*0.3);
        armHeight += pct;

        SmartDashboard.putNumber("armHeight ", armHeight);
        SmartDashboard.putNumber("armEncoderReadout2 ", mRightArmEncoder.getPosition());
        
        
        mRightArmPIDController.setReference(armHeight, CANSparkMax.ControlType.kPosition);
        // mRightArmPIDController.setReference(-0.1, CANSparkMax.ControlType.kSmartMotion);

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm... Angle?", mRightArmEncoder.getPosition());
    }

    public void stop() {
       mRightArmMotor.set(0);
            
   }

}