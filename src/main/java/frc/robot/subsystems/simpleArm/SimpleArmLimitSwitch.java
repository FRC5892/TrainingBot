// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.simpleArm;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.util.LoggedDIO.SimDIO;

/** Add your docs here. */
public class SimpleArmLimitSwitch extends SimDIO {

    public SimpleArmLimitSwitch(String name, SimpleArmSim armSim) {
        super(name, () -> {
            final var angle = MathUtil.angleModulus(armSim.getPosition().in(Radians));
            return (angle >= Units.degreesToRadians(44)) && (angle <= Units.degreesToRadians(50));
        });
    }
}
