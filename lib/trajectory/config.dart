import 'package:flutter/material.dart';
import 'package:pathplanner/trajectory/dc_motor.dart';
import 'package:pathplanner/util/prefs.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';
import 'package:pathplanner/util/wpimath/kinematics.dart';
import 'package:shared_preferences/shared_preferences.dart';

class RobotConfig {
  final num massKG;
  final num moi;
  final Size bumperSize;
  final Translation2d bumperOffset;
  final ModuleConfig moduleConfig;
  final List<Translation2d> moduleLocations;
  final bool holonomic;
  late final SwerveDriveKinematics kinematics;

  RobotConfig({
    required this.massKG,
    required this.moi,
    required this.bumperSize,
    required this.bumperOffset,
    required this.moduleConfig,
    required this.moduleLocations,
    required this.holonomic,
  }) : kinematics = SwerveDriveKinematics(moduleLocations);

  factory RobotConfig.fromPrefs(SharedPreferences prefs) {
    bool holonomicMode =
        prefs.getBool(PrefsKeys.holonomicMode) ?? Defaults.holonomicMode;
    int numMotors = holonomicMode ? 1 : 2;
    ModuleConfig moduleConfig = ModuleConfig.fromPrefs(prefs, numMotors);
    num halfTrackwidth = (prefs.getDouble(PrefsKeys.robotTrackwidth) ??
            Defaults.robotTrackwidth) /
        2;
    List<Translation2d> moduleLocations = holonomicMode
        ? [
            Translation2d(
                prefs.getDouble(PrefsKeys.flModuleX) ?? Defaults.flModuleX,
                prefs.getDouble(PrefsKeys.flModuleY) ?? Defaults.flModuleY),
            Translation2d(
                prefs.getDouble(PrefsKeys.frModuleX) ?? Defaults.frModuleX,
                prefs.getDouble(PrefsKeys.frModuleY) ?? Defaults.frModuleY),
            Translation2d(
                prefs.getDouble(PrefsKeys.blModuleX) ?? Defaults.blModuleX,
                prefs.getDouble(PrefsKeys.blModuleY) ?? Defaults.blModuleY),
            Translation2d(
                prefs.getDouble(PrefsKeys.brModuleX) ?? Defaults.brModuleX,
                prefs.getDouble(PrefsKeys.brModuleY) ?? Defaults.brModuleY),
          ]
        : [
            Translation2d(0, halfTrackwidth),
            Translation2d(0, -halfTrackwidth),
          ];
    Size bumperSize = Size(
        prefs.getDouble(PrefsKeys.robotWidth) ?? Defaults.robotWidth,
        prefs.getDouble(PrefsKeys.robotLength) ?? Defaults.robotLength);
    Translation2d bumperOffset = Translation2d(
        prefs.getDouble(PrefsKeys.bumperOffsetX) ?? Defaults.bumperOffsetX,
        prefs.getDouble(PrefsKeys.bumperOffsetY) ?? Defaults.bumperOffsetY);

    return RobotConfig(
      massKG: prefs.getDouble(PrefsKeys.robotMass) ?? Defaults.robotMass,
      moi: prefs.getDouble(PrefsKeys.robotMOI) ?? Defaults.robotMOI,
      bumperSize: bumperSize,
      bumperOffset: bumperOffset,
      moduleConfig: moduleConfig,
      moduleLocations: moduleLocations,
      holonomic: holonomicMode,
    );
  }

  /// Build a [RobotConfig] from a foreign project's settings.json map.
  /// Uses the same keys as [SharedPreferences] but reads directly from JSON.
  factory RobotConfig.fromSettingsJson(Map<String, dynamic> json) {
    bool holonomicMode =
        json[PrefsKeys.holonomicMode] ?? Defaults.holonomicMode;
    int numMotors = holonomicMode ? 1 : 2;

    num halfTrackwidth =
        ((json[PrefsKeys.robotTrackwidth] as num?)?.toDouble() ??
                Defaults.robotTrackwidth) /
            2;

    ModuleConfig moduleConfig = ModuleConfig(
      wheelRadiusMeters:
          (json[PrefsKeys.driveWheelRadius] as num?)?.toDouble() ??
              Defaults.driveWheelRadius,
      maxDriveVelocityMPS:
          (json[PrefsKeys.maxDriveSpeed] as num?)?.toDouble() ??
              Defaults.maxDriveSpeed,
      driveMotor: DCMotor.fromString(
              json[PrefsKeys.driveMotor] ?? Defaults.driveMotor, numMotors)
          .withReduction(
              (json[PrefsKeys.driveGearing] as num?)?.toDouble() ??
                  Defaults.driveGearing),
      driveCurrentLimit: ((json[PrefsKeys.driveCurrentLimit] as num?)
                  ?.toDouble() ??
              Defaults.driveCurrentLimit) *
          numMotors,
      wheelCOF:
          (json[PrefsKeys.wheelCOF] as num?)?.toDouble() ?? Defaults.wheelCOF,
    );

    List<Translation2d> moduleLocations = holonomicMode
        ? [
            Translation2d(
                (json[PrefsKeys.flModuleX] as num?)?.toDouble() ??
                    Defaults.flModuleX,
                (json[PrefsKeys.flModuleY] as num?)?.toDouble() ??
                    Defaults.flModuleY),
            Translation2d(
                (json[PrefsKeys.frModuleX] as num?)?.toDouble() ??
                    Defaults.frModuleX,
                (json[PrefsKeys.frModuleY] as num?)?.toDouble() ??
                    Defaults.frModuleY),
            Translation2d(
                (json[PrefsKeys.blModuleX] as num?)?.toDouble() ??
                    Defaults.blModuleX,
                (json[PrefsKeys.blModuleY] as num?)?.toDouble() ??
                    Defaults.blModuleY),
            Translation2d(
                (json[PrefsKeys.brModuleX] as num?)?.toDouble() ??
                    Defaults.brModuleX,
                (json[PrefsKeys.brModuleY] as num?)?.toDouble() ??
                    Defaults.brModuleY),
          ]
        : [
            Translation2d(0, halfTrackwidth),
            Translation2d(0, -halfTrackwidth),
          ];

    Size bumperSize = Size(
        (json[PrefsKeys.robotWidth] as num?)?.toDouble() ?? Defaults.robotWidth,
        (json[PrefsKeys.robotLength] as num?)?.toDouble() ??
            Defaults.robotLength);
    Translation2d bumperOffset = Translation2d(
        (json[PrefsKeys.bumperOffsetX] as num?)?.toDouble() ??
            Defaults.bumperOffsetX,
        (json[PrefsKeys.bumperOffsetY] as num?)?.toDouble() ??
            Defaults.bumperOffsetY);

    return RobotConfig(
      massKG: (json[PrefsKeys.robotMass] as num?)?.toDouble() ??
          Defaults.robotMass,
      moi: (json[PrefsKeys.robotMOI] as num?)?.toDouble() ?? Defaults.robotMOI,
      bumperSize: bumperSize,
      bumperOffset: bumperOffset,
      moduleConfig: moduleConfig,
      moduleLocations: moduleLocations,
      holonomic: holonomicMode,
    );
  }
}

class ModuleConfig {
  final num wheelRadiusMeters;
  final num maxDriveVelocityMPS;
  final DCMotor driveMotor;
  final num driveCurrentLimit;
  final num wheelCOF;

  const ModuleConfig({
    required this.wheelRadiusMeters,
    required this.maxDriveVelocityMPS,
    required this.driveMotor,
    required this.driveCurrentLimit,
    required this.wheelCOF,
  });

  ModuleConfig.fromPrefs(SharedPreferences prefs, int numMotors)
      : this(
          wheelRadiusMeters: prefs.getDouble(PrefsKeys.driveWheelRadius) ??
              Defaults.driveWheelRadius,
          maxDriveVelocityMPS: prefs.getDouble(PrefsKeys.maxDriveSpeed) ??
              Defaults.maxDriveSpeed,
          driveMotor: DCMotor.fromString(
                  prefs.getString(PrefsKeys.driveMotor) ?? Defaults.driveMotor,
                  numMotors)
              .withReduction(prefs.getDouble(PrefsKeys.driveGearing) ??
                  Defaults.driveGearing),
          driveCurrentLimit: (prefs.getDouble(PrefsKeys.driveCurrentLimit) ??
                  Defaults.driveCurrentLimit) *
              numMotors,
          wheelCOF: prefs.getDouble(PrefsKeys.wheelCOF) ?? Defaults.wheelCOF,
        );

  num get maxDriveVelocityRadPerSec => maxDriveVelocityMPS / wheelRadiusMeters;
}
