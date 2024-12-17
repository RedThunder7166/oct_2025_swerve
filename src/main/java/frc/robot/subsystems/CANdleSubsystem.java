// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

public class CANdleSubsystem extends SubsystemBase {
    private final CANdle m_candle = new CANdle(Constants.CANDLE_ID, "HighVoltageCANivore");
    private final int LedCount = Constants.CANDLE_LED_COUNT;

    private final Animation m_color_flow_animation = new ColorFlowAnimation(128, 20, 70, 0, 0.7, LedCount, Direction.Forward);
    private final Animation m_fire_animation = new FireAnimation(0.5, 0.7, LedCount, 0.7, 0.5);
    private final Animation m_larson_animation = new LarsonAnimation(0, 255, 46, 0, 1, LedCount, BounceMode.Front, 3);
    private final Animation m_rainbow_animation = new RainbowAnimation(1, 0.1, LedCount);
    private final Animation m_rgbfade_animation = new RgbFadeAnimation(0.7, 0.4, LedCount);
    private final Animation m_single_fade_animation = new SingleFadeAnimation(50, 2, 200, 0, 0.5, LedCount);
    private final Animation m_strobe_animation = new StrobeAnimation(240, 10, 180, 0, 98.0 / 256.0, LedCount);
    private final Animation m_twinkle_animation = new TwinkleAnimation(30, 70, 60, 0, 0.4, LedCount, TwinklePercent.Percent6);
    private final Animation m_twinkle_off_animation = new TwinkleOffAnimation(70, 90, 175, 0, 0.8, LedCount, TwinkleOffPercent.Percent100);

    private Animation m_animation = m_color_flow_animation;

    public CANdleSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.statusLedOffWhenActive = true;
        config.disableWhenLOS = false;
        config.stripType = LEDStripType.RGB;
        config.brightnessScalar = 0.1;
        config.vBatOutputMode = VBatOutputMode.Modulated;

        m_candle.configAllSettings(config, 100);
    }

    public void incrementAnimation() {
        if (m_animation == m_color_flow_animation) changeAnimation(m_fire_animation);
        else if (m_animation == m_fire_animation) changeAnimation(m_larson_animation);
        else if (m_animation == m_larson_animation) changeAnimation(m_rainbow_animation);
        else if (m_animation == m_rainbow_animation) changeAnimation(m_rgbfade_animation);
        else if (m_animation == m_rgbfade_animation) changeAnimation(m_single_fade_animation);
        else if (m_animation == m_single_fade_animation) changeAnimation(m_strobe_animation);
        else if (m_animation == m_strobe_animation) changeAnimation(m_twinkle_animation);
        else if (m_animation == m_twinkle_animation) changeAnimation(m_twinkle_off_animation);
        else if (m_animation == m_twinkle_off_animation) changeAnimation(m_color_flow_animation);
    }
    public void decrementAnimation() {
        if (m_animation == m_color_flow_animation) changeAnimation(m_twinkle_off_animation);
        else if (m_animation == m_fire_animation) changeAnimation(m_color_flow_animation);
        else if (m_animation == m_larson_animation) changeAnimation(m_fire_animation);
        else if (m_animation == m_rainbow_animation) changeAnimation(m_larson_animation);
        else if (m_animation == m_rgbfade_animation) changeAnimation(m_rainbow_animation);
        else if (m_animation == m_single_fade_animation) changeAnimation(m_rgbfade_animation);
        else if (m_animation == m_strobe_animation) changeAnimation(m_single_fade_animation);
        else if (m_animation == m_twinkle_animation) changeAnimation(m_strobe_animation);
        else if (m_animation == m_twinkle_off_animation) changeAnimation(m_twinkle_animation);
    }

    public void changeAnimation(Animation new_animation) {
        m_animation = new_animation;
    }

    @Override
    public void periodic() {
        m_candle.animate(m_animation);
    }

    @Override
    public void simulationPeriodic() {
    }
}
