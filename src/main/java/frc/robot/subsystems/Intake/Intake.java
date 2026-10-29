package frc.robot.subsystems.Intake;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.TalonFX.LoggedTalonFX;
import java.util.function.DoubleSupplier;

public class Intake extends SubsystemBase {

    private static final double DEPLOY_SPEED = 0.25;
    private static final double Roller_SPEED = 0.5;

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0);

    private final LoggedTalonFX deployMotor;
    private final LoggedTalonFX rollerMotor;

    public Intake(LoggedTalonFX deployMotor, LoggedTalonFX rollerMotor) {
        this.deployMotor = deployMotor;
        this.rollerMotor = rollerMotor;
        this.deployMotor.withConfig(LoggedTalonFX.buildStandardConfig(40, 20, NeutralModeValue.Brake)
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(10))
        );
        this.rollerMotor.withConfig(LoggedTalonFX.buildStandardConfig(80, 40));
    }

    public Command intakeSequence() {
        return startEnd(
                        () -> deployMotor.setControl(dutyCycleOut.withOutput(DEPLOY_SPEED)),
                        () -> deployMotor.setControl(dutyCycleOut.withOutput(0)))
                .until(() -> deployMotor.getPosition().in(Rotations) >= 0.125)
                .andThen(startEnd(
                        () -> rollerMotor.setControl(dutyCycleOut.withOutput(Roller_SPEED)),
                        () -> rollerMotor.setControl(dutyCycleOut.withOutput(0))));
    }

    public Command retract() {
        return startEnd(
                        () -> deployMotor.setControl(dutyCycleOut.withOutput(-DEPLOY_SPEED)),
                        () -> deployMotor.setControl(dutyCycleOut.withOutput(0)))
                .until(() -> deployMotor.getPosition().in(Rotations) <= 0.01);
    }

    public Command joystickControl(DoubleSupplier joystickPosition) {
        return runEnd(
                () -> rollerMotor.setControl(dutyCycleOut.withOutput(joystickPosition.getAsDouble())),
                () -> rollerMotor.setControl(dutyCycleOut.withOutput(0)));
    }

    @Override
    public void periodic() {
        deployMotor.periodic();
        rollerMotor.periodic();
    }
}
