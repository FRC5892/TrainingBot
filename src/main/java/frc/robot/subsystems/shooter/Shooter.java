package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;

public class Shooter extends SubsystemBase {

    private final LoggedTalonFX motor;

    private final DutyCycleOut control = new DutyCycleOut(0);

    private final double SHOOTING_SPEED = 0.4;

    public Shooter(LoggedTalonFX motor) {
        this.motor = motor;
        motor.withConfig(LoggedTalonFX.buildStandardConfig(80, 60));
    }

    public Command runShooter() {
        return runEnd(
                () -> motor.setControl(control.withOutput(SHOOTING_SPEED)),
                () -> motor.setControl(control.withOutput(0)));
    }

    @Override
    public void periodic() {
        motor.periodic();
    }
}
