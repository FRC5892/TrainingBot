package frc.robot.subsystems.Intake.sim;

import com.ctre.phoenix6.CANBus;
import frc.robot.util.LoggedTalon.Follower.PhoenixTalonFollower;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXSimpleMotorSim;
import frc.robot.util.LoggedTalon.TalonInputs;

public class IntakeRollerSim extends TalonFXSimpleMotorSim {

    @SuppressWarnings("unused")
    public IntakeRollerSim(int canID, CANBus canBus, String name, PhoenixTalonFollower... followers) {
        super(canID, canBus, name, 1, IntakeSimConstants.LESSON == 0 ? 10 : 50, followers);
    }

    @Override
    protected void simulationPeriodic(TalonInputs inputs) {
        super.simulationPeriodic(inputs);
        IntakeSimState.getInstance().updateRollerRotation(inputs.positionRot);
    }
}
