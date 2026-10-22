package frc.robot.subsystems.simpleArm;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;

public class SimpleArm extends SubsystemBase {
    private final LoggedTalonFX motor;
    private final DutyCycleOut out = new DutyCycleOut(0);

    public SimpleArm(LoggedTalonFX motor) {
        this.motor = motor;
        motor.withConfig(LoggedTalonFX.buildStandardConfig(80, 40));
    }

    public Command spinForward() {
        return startEnd(() -> motor.setControl(out.withOutput(0.5)), () -> motor.setControl(out.withOutput(0)))
                .until(() -> motor.getPosition().in(Rotations) >= 0.09);
    }
    public Command spinBackward() {
        return startEnd(() -> motor.setControl(out.withOutput(-0.5)), () -> motor.setControl(out.withOutput(0)))
                .until(() -> motor.getPosition().in(Rotations) <= -0.9);
    }
    @Override
    public void periodic() {
        motor.periodic();
    }
}
