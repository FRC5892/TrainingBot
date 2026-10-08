package frc.robot.subsystems.simpleArm;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;

public class SimpleArm extends SubsystemBase {
    private final LoggedTalonFX motor;
    private final DutyCycleOut request = new DutyCycleOut(0);

    public SimpleArm(LoggedTalonFX motor) {
        this.motor = motor;
        motor.withConfig(LoggedTalonFX.buildStandardConfig(80, 60));
    }

    public Command spinCommand() {
        return runEnd(() -> motor.setControl(request.withOutput(0.5)), () -> motor.setControl(request.withOutput(0)));
    }

    @Override
    public void periodic() {
        motor.periodic();
    }
}
