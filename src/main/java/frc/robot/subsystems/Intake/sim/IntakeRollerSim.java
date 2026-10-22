package frc.robot.subsystems.Intake.sim;

import com.ctre.phoenix6.CANBus;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXSimpleMotorSim;
import frc.robot.util.LoggedTalon.TalonInputs;

public class IntakeRollerSim extends TalonFXSimpleMotorSim {

    public IntakeRollerSim(int canID, CANBus canBus, String name) {
        super(canID, canBus, name, 1, 25);
    }

    @Override
    protected void simulationPeriodic(TalonInputs inputs) {
        super.simulationPeriodic(inputs);
        IntakeSimState.getInstance().updateRollerRotation(inputs.positionRot);
    }
}
