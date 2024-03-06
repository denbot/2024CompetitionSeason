package frc.robot.generated;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;

public class ArmTunerConstants {

  // TODO: Tune motionmagic and pid vals later
  public static final MotionMagicConfigs pivotMotionMagicConfigs = new MotionMagicConfigs()
    .withMotionMagicAcceleration(2)
    .withMotionMagicCruiseVelocity(1);

  public static final Slot0Configs pivotPIDConfigs = new Slot0Configs()
    .withKP(4.8)
    .withKD(0.1);
}
