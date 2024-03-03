package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    CANSparkMax topSparkMax = new CANSparkMax(ShooterConstants.topMotorID, MotorType.kBrushless);
    CANSparkMax bottomSparkMax = new CANSparkMax(ShooterConstants.bottomMotorID, MotorType.kBrushless);

    SparkMaxPIDController topController = topSparkMax.getPIDController();
    SparkMaxPIDController bottomController = bottomSparkMax.getPIDController();


    public Shooter() {
        topSparkMax.setIdleMode(IdleMode.kCoast);
        topSparkMax.setInverted(ShooterConstants.kTopInverted);
        topSparkMax.setSmartCurrentLimit(ShooterConstants.stallCurentLimit,
                ShooterConstants.freeCurentLimit);
        topController.setP(ShooterConstants.FlywheelPIDF.kP,0);
        topController.setI(ShooterConstants.FlywheelPIDF.kI,0);
        topController.setD(ShooterConstants.FlywheelPIDF.kD,0);
        topController.setFF(ShooterConstants.FlywheelPIDF.kF,0);


        bottomSparkMax.setIdleMode(IdleMode.kCoast);
        bottomSparkMax.setInverted(ShooterConstants.kBottomInverted);
        bottomSparkMax.setSmartCurrentLimit(ShooterConstants.stallCurentLimit,
                ShooterConstants.freeCurentLimit);
        setDefaultCommand(new RunCommand(this::stop, this));
    }

    private void setSpeeds(double topSpeed, double bottomSpeed) {
        topController.setReference(topSpeed,CANSparkMax.ControlType.kVelocity);
        bottomController.setReference(bottomSpeed,CANSparkMax.ControlType.kVelocity);
    }

    public void run() {
        setSpeeds(ShooterConstants.topMotorSpeed,
                ShooterConstants.bottomMotorSpeed);
    }

    public void stop() {
        topSparkMax.stopMotor();
        bottomSparkMax.stopMotor();
    }
}
