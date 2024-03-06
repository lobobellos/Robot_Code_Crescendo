package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmPivotConstants;

public class ArmPivot extends SubsystemBase {

    private final CANSparkMax motor = new CANSparkMax(ArmPivotConstants.ID, MotorType.kBrushless);

    private final AnalogEncoder encoder = new AnalogEncoder(ArmPivotConstants.encoderPort);

    private final PIDController controller = new PIDController(
            ArmPivotConstants.ArmPID.kP,
            ArmPivotConstants.ArmPID.kI,
            ArmPivotConstants.ArmPID.kD);

    private Rotation2d setpoint = Rotation2d.fromDegrees(0);

    public ArmPivot(){
        setSetpoint(Rotation2d.fromDegrees(0));
    }

    public void setSetpoint(Rotation2d setpoint) {
        this.setpoint = setpoint;
    }

    public void run() {
        double output = controller.calculate(getRotation().getDegrees(), setpoint.getDegrees());
        motor.set(output);
    }

    private Rotation2d getRotation() {
        return Rotation2d.fromRotations(encoder.get())
                .plus(ArmPivotConstants.encoderOffset);
    }

    public Command setSetpointCommand(Rotation2d setpoint) {
        return Commands.runOnce(() -> this.setSetpoint(setpoint));
    }

    public Command runCommand(){
        return Commands.run(this::run,this);
    }
}
