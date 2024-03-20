package frc.robot.subsystems;

import java.util.HashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPivotConstants;
import frc.robot.Constants.ArmPivotConstants.jointPosition;

public class ArmPivot extends SubsystemBase {

    private final CANSparkMax motor = new CANSparkMax(ArmPivotConstants.ID, MotorType.kBrushless);

    //private final AnalogEncoder encoder = new AnalogEncoder(ArmPivotConstants.encoderPort);

    private final DutyCycleEncoder encoder= new DutyCycleEncoder(8);

    private final PIDController controller = new PIDController(
            ArmPivotConstants.ArmPID.kP,
            ArmPivotConstants.ArmPID.kI,
            ArmPivotConstants.ArmPID.kD);


    private HashMap<jointPosition,Rotation2d> jointPos = new HashMap<jointPosition,Rotation2d>();


    ShuffleboardTab tab;

    jointPosition position;

    public ArmPivot(){
        encoder.reset();
        position = jointPosition.starting;

        tab = Shuffleboard.getTab("armPivot");
        tab.add("armPivot",controller);
        tab.addDouble("position degs", getRotation()::getDegrees);

        setDefaultCommand(this.runCommand());

        jointPos.put(jointPosition.starting, Rotation2d.fromDegrees(0));
        jointPos.put(jointPosition.mid, ArmPivotConstants.midPosition);
        jointPos.put(jointPosition.high, ArmPivotConstants.highPosition);

    }

    public void run() {
        Rotation2d setpoint = jointPos.getOrDefault(position,Rotation2d.fromDegrees(0));
        double output = controller.calculate(getRotation().getDegrees(), setpoint.getDegrees());
        motor.set(output);
    }

    private Rotation2d getRotation() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition())
                .plus(ArmPivotConstants.encoderOffset);
    }

    public void setPosition(jointPosition position) {
        this.position = position;
    }

    public Command setPositionCommand(jointPosition pos) {
        return Commands.runOnce(() -> this.setPosition(pos));
    }

    public Command runCommand(){
        return Commands.run(this::run,this);
    }

    public void toggleJoint(){
        if(position == jointPosition.starting){
            position = jointPosition.mid;
        }else if(position == jointPosition.mid ){
            position = jointPosition.high;
        }else if(position == jointPosition.high){
            position = jointPosition.starting;
        }
    }

    public Command ToggleJointCommand(){
        return Commands.runOnce(this::toggleJoint);
    }

    public void periodic(){
        SmartDashboard.putNumber("pivot/pos", getRotation().getDegrees());
    }
}
