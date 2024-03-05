package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

    private final CANSparkMax topSparkMax = new CANSparkMax(ShooterConstants.topMotorID, MotorType.kBrushless);
    private final CANSparkMax bottomSparkMax = new CANSparkMax(ShooterConstants.bottomMotorID, MotorType.kBrushless);

    private final SparkMaxPIDController topController = topSparkMax.getPIDController();
    private final SparkMaxPIDController bottomController = bottomSparkMax.getPIDController();

    private final RelativeEncoder topEncoder = topSparkMax.getEncoder();
    private final RelativeEncoder bottomEncoder = bottomSparkMax.getEncoder();

    private double topSetpoint;
    private double bottomSetpoint;

    private boolean speedSet = false;

    public Shooter() {
        topSparkMax.setIdleMode(IdleMode.kCoast);
        topSparkMax.setInverted(ShooterConstants.kTopInverted);
        topSparkMax.setSmartCurrentLimit(ShooterConstants.stallCurentLimit,
                ShooterConstants.freeCurentLimit);
        topController.setP(ShooterConstants.FlywheelPIDF.kP, 0);
        topController.setI(ShooterConstants.FlywheelPIDF.kI, 0);
        topController.setD(ShooterConstants.FlywheelPIDF.kD, 0);
        topController.setFF(ShooterConstants.FlywheelPIDF.kF, 0);

        bottomSparkMax.setIdleMode(IdleMode.kCoast);
        bottomSparkMax.setInverted(ShooterConstants.kBottomInverted);
        bottomSparkMax.setSmartCurrentLimit(ShooterConstants.stallCurentLimit,
                ShooterConstants.freeCurentLimit);
        setDefaultCommand(new RunCommand(this::stop, this));

        var tab = Shuffleboard.getTab("shooter");

        tab.addDouble("top velocity", () -> topEncoder.getVelocity());
        tab.addDouble("topSetpoint", ()->topSetpoint);
    }

    private void setSpeeds(double topSpeed, double bottomSpeed) {
        topSetpoint = topSpeed;
        bottomSetpoint = bottomSpeed;
        speedSet = true;
        topController.setReference(topSpeed, CANSparkMax.ControlType.kVelocity);
        bottomController.setReference(bottomSpeed, CANSparkMax.ControlType.kVelocity);
    }

    public void runAmp() {
        setSpeeds(ShooterConstants.ampTopMotorSpeed,
                ShooterConstants.ampBottomMotorSpeed);
    }

    public void runSpeaker(){
        setSpeeds(ShooterConstants.speakerTopMotorSpeed,
        ShooterConstants.speakerBottomMotorSpeed );
    }

    public void stop() {
        topSetpoint = 0;
        bottomSetpoint = 0;
        topSparkMax.stopMotor();
        bottomSparkMax.stopMotor();
    }

    public Command runAmpCommand() {
        return Commands.run(this::runAmp, this);
    }

    public Command runSpeakerCommand(){
        return Commands.run(this::runSpeaker,this);
    }

    public boolean atSetpoint() {
        if (!speedSet)
            return false;
        return Math.abs(topEncoder.getVelocity() - topSetpoint) < ShooterConstants.speedTolerance &&
                Math.abs(bottomEncoder.getVelocity() - bottomSetpoint) < ShooterConstants.speedTolerance;
    }

    public void periodic(){
        SmartDashboard.putNumberArray("shooter speed", 
        new double[] {topEncoder.getVelocity(), topSetpoint });
    }
}
