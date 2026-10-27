package frc.robot.subsystems.simpleArm;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedDIO.LoggedDIO;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;

public class SimpleArm extends SubsystemBase {
    private final LoggedTalonFX motor;
    private final LoggedDIO limit;

    private final DutyCycleOut out = new DutyCycleOut(0);

    public SimpleArm(LoggedTalonFX motor,LoggedDIO limit) {
        this.motor = motor;
        this.limit = limit;
        motor.withConfig(LoggedTalonFX.buildStandardConfig(80, 40, NeutralModeValue.Brake));
    }

    public Command spinForward() {
        return startEnd(() -> motor.setControl(out.withOutput(0.5)), () -> motor.setControl(out.withOutput(0)))
                .until(limit);
    }

    public Command spinBackward() {
        return startEnd(() -> motor.setControl(out.withOutput(-0.5)), () -> motor.setControl(out.withOutput(0)))
                .until(() -> motor.getPosition().in(Rotations) <= -0.9);
    }

    @Override
    public void periodic() {
        motor.periodic();
        limit.periodic();
    }
}
