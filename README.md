# Swerve Test

This repo contains the code used to make the swerve drive base for the 2024 FRC season. Developed by [BakedPotatoLord](https://github.com/bakedpotatolord)

Hardware components for this year include:
- SDS mk4 swerve modules
- REV NEO Brushless 1.1 drive motor
- REV NEO Brushless 1.1 steering motor
- ThriftyBot Analog steering encoder

## Vendor Dependencies
- REV: `https://software-metadata.revrobotics.com/REVLib-2024.json`
- Phoenix 5: `https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2024-beta-latest.json`
- Phoenix 6: `https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2024-beta-latest.json`
- ADIS16470: `https://maven.highcurrent.io/vendordeps/ADIS16470.json`
- WPILIB New Commands


## Button Bindings

|Button/Axis|What It Do|
|---|---|
|Left X Axis|X Speed|
|Left Y Axis |Y Speed|
|Right X Axis|Rotation Speed|
|ButtonA|Re-Zero Gyro|
|ButtonL|Turn 90deg to left|
|ButtonR|Turn 90deg to right|
|ButtonR+ButtonL+ButtonY|Toggle Demo Mode (terrifying)|