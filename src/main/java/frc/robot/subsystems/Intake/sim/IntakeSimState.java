// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake.sim;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

/** Add your docs here. */
public class IntakeSimState {

    private static IntakeSimState instance;

    private LoggedMechanism2d mech = new LoggedMechanism2d(2, 2);
    private LoggedMechanismRoot2d root = mech.getRoot("motor", 1, 1);
    private LoggedMechanismLigament2d intake = root.append(
                    new LoggedMechanismLigament2d("chassis", 0.4, 180, 5, new Color8Bit(128, 128, 128)))
            .append(new LoggedMechanismLigament2d("bumper", 0.4, 90, 5, new Color8Bit(212, 6, 6)));
    private LoggedMechanismLigament2d arm = root.append(
                    new LoggedMechanismLigament2d("armStub", 0.1, 90, 5, new Color8Bit(128, 128, 128)))
            .append(new LoggedMechanismLigament2d("arm", 0.6, 0));
    private LoggedMechanismLigament2d roller =
            arm.append(new LoggedMechanismLigament2d("roller1", 0.1, 90, 7, new Color8Bit(0, 255, 0)));
    private LoggedMechanismLigament2d roller2 =
            roller.append(new LoggedMechanismLigament2d("roller2", 0.2, 180, 7, new Color8Bit(0, 255, 0)));

    public static IntakeSimState getInstance() {
        if (instance == null) {
            instance = new IntakeSimState();
        }
        return instance;
    }

    public void updateDeployRotation(double positionRotations) {
        arm.setAngle(Units.rotationsToDegrees(positionRotations + 0.125));
    }

    public void updateRollerRotation(double positionRotations) {
        roller.setAngle(Units.rotationsToDegrees(positionRotations));
    }

    public void log() {
        Logger.recordOutput("Intake/mech", mech);
    }
}
