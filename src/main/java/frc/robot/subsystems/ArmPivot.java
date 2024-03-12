package frc.robot.subsystems;

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

public class ArmPivot extends SubsystemBase {

    private final CANSparkMax motor = new CANSparkMax(ArmPivotConstants.ID, MotorType.kBrushless);

    //private final AnalogEncoder encoder = new AnalogEncoder(ArmPivotConstants.encoderPort);

    private final DutyCycleEncoder encoder= new DutyCycleEncoder(8);

    private final PIDController controller = new PIDController(
            ArmPivotConstants.ArmPID.kP,
            ArmPivotConstants.ArmPID.kI,
            ArmPivotConstants.ArmPID.kD);

    private Rotation2d setpoint = Rotation2d.fromDegrees(0);

    ShuffleboardTab tab;

    double output;

    public ArmPivot(){
        setSetpoint(Rotation2d.fromDegrees(0));
        encoder.reset();
        

        tab = Shuffleboard.getTab("armPivot");
        tab.add("armPivot",controller);
        tab.addDouble("pid output", this::getOutput);
        tab.addDouble("position degs", getRotation()::getDegrees);

        setDefaultCommand(this.runCommand());
    }

    public void setSetpoint(Rotation2d setpoint) {
        this.setpoint = setpoint;
    }

    public void run() {
        output = controller.calculate(getRotation().getDegrees(), setpoint.getDegrees());
        motor.set(output);
    }

    private Rotation2d getRotation() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition())
                .plus(ArmPivotConstants.encoderOffset);
    }

    public Command setSetpointCommand(Rotation2d setpoint) {
        return Commands.runOnce(() -> this.setSetpoint(setpoint));
    }

    public Command runCommand(){
        return Commands.run(this::run,this);
    }

    private double getOutput(){
        return output;
    }

    public void toggleJoint(){
        if(setpoint.equals(Rotation2d.fromDegrees(0))){
            setpoint = ArmPivotConstants.ampScoringPosition;
        }else if(setpoint.equals(ArmPivotConstants.ampScoringPosition)){
            setpoint = ArmPivotConstants.speakerScoringPosition;
        }
        else if(setpoint.equals(ArmPivotConstants.speakerScoringPosition)){
            setpoint = Rotation2d.fromDegrees(0);
        }
    }

    public Command ToggleJointCommand(){
        return Commands.runOnce(this::toggleJoint);
    }

    public void periodic(){
        SmartDashboard.putNumber("pivot/pos", getRotation().getDegrees());
        SmartDashboard.putNumber("pivot/setPoint", setpoint.getDegrees());
        SmartDashboard.putNumber("pivot/PID output",getOutput());
        Shuffleboard.update();

    }
}
