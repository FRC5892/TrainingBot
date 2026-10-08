package frc.robot.subsystems.simpleArm;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedDIO.LoggedDIO;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;

public class SimpleArm extends SubsystemBase {
    private final LoggedTalonFX motor;
    private final LoggedDIO limitSwitch;
    private final DutyCycleOut request = new DutyCycleOut(0);

    public SimpleArm(LoggedTalonFX motor, LoggedDIO limitSwitch) {
        this.motor = motor;
        this.limitSwitch = limitSwitch;
        motor.withConfig(LoggedTalonFX.buildStandardConfig(80, 60));
    }

    public Command spinCommand() {
        return runEnd(() -> motor.setControl(request.withOutput(0.25)), () -> motor.setControl(request.withOutput(0)))
                .until(limitSwitch::get);
    }

    @Override
    public void periodic() {
        motor.periodic();
        limitSwitch.periodic();
    }
}
