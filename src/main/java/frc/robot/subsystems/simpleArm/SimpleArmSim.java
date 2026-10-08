package frc.robot.subsystems.simpleArm;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedTalon.TalonFX.TalonFXSimpleMotorSim;
import frc.robot.util.LoggedTalon.TalonInputs;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class SimpleArmSim extends TalonFXSimpleMotorSim {

    private LoggedMechanism2d mech = new LoggedMechanism2d(2, 2);
    private LoggedMechanismRoot2d root = mech.getRoot("motor", 1, 1);
    private LoggedMechanismLigament2d limit = root.append(new LoggedMechanismLigament2d("limitArm", 0.6, 45))
            .append(new LoggedMechanismLigament2d("limit", 0.1, -90));
    private LoggedMechanismLigament2d arm = root.append(new LoggedMechanismLigament2d("arm", 0.5, 0));

    public SimpleArmSim(int canID, CANBus canBus, String name) {
        super(canID, canBus, name, 2.5, 25);
    }

    @Override
    protected void simulationPeriodic(TalonInputs inputs) {
        super.simulationPeriodic(inputs);
        arm.setAngle(Units.rotationsToDegrees(inputs.positionRot));
        Logger.recordOutput("SimpleMotor/mech", mech);
    }
}
