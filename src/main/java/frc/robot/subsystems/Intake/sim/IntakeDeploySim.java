package frc.robot.subsystems.Intake.sim;

import com.ctre.phoenix6.CANBus;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXSimpleMotorSim;
import frc.robot.util.LoggedTalon.TalonInputs;

public class IntakeDeploySim extends TalonFXSimpleMotorSim {

    public IntakeDeploySim(int canID, CANBus canBus, String name) {
        super(canID, canBus, name, 2, 50);
    }

    @Override
    protected void simulationPeriodic(TalonInputs inputs) {
        super.simulationPeriodic(inputs);
        IntakeSimState.getInstance().updateDeployRotation(inputs.positionRot);
    }
}
