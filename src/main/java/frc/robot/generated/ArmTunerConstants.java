package frc.robot.generated;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class ArmTunerConstants {

  // TODO: Tune motionmagic and pid vals later
  public static final MotionMagicConfigs pivotMotionMagicConfigs = new MotionMagicConfigs()
    .withMotionMagicAcceleration(2)
    .withMotionMagicCruiseVelocity(1);

  public static final Slot0Configs pivotPIDConfigs = new Slot0Configs()
    .withKP(4.8)
    .withKD(0.1)
    .withGravityType(GravityTypeValue.Arm_Cosine)
    .withKG(0); // tune later so it hovers horizontal (may need to be negative) 0 has to be straight out
}
