import 'dart:convert';
import 'dart:io';

import 'package:file_selector/file_selector.dart';
import 'package:flutter/material.dart';
import 'package:pathplanner/services/log.dart';
import 'package:pathplanner/trajectory/trajectory.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';
import 'package:pathplanner/util/wpimath/kinematics.dart';

/// A ghost auto represents a previously exported auto's simulated trajectory
/// along with the robot configuration used to generate it. This allows
/// visualizing another robot's (or the same robot's) auto as a translucent
/// reference overlay while editing a different auto.
class GhostAuto {
  final String name;
  final PathPlannerTrajectory trajectory;
  final Size bumperSize;
  final Translation2d bumperOffset;
  final List<Translation2d> moduleLocations;
  final bool holonomic;

  const GhostAuto({
    required this.name,
    required this.trajectory,
    required this.bumperSize,
    required this.bumperOffset,
    required this.moduleLocations,
    required this.holonomic,
  });

  /// Serialize the ghost auto to JSON.
  Map<String, dynamic> toJson() {
    return {
      'version': '1.0',
      'name': name,
      'bumperSize': {
        'width': bumperSize.width,
        'height': bumperSize.height,
      },
      'bumperOffset': {
        'x': bumperOffset.x,
        'y': bumperOffset.y,
      },
      'holonomic': holonomic,
      'moduleLocations': [
        for (Translation2d loc in moduleLocations)
          {'x': loc.x, 'y': loc.y},
      ],
      'states': [
        for (TrajectoryState s in trajectory.states)
          {
            't': s.timeSeconds,
            'x': s.pose.x,
            'y': s.pose.y,
            'rotation': s.pose.rotation.radians,
            'vx': s.fieldSpeeds.vx,
            'vy': s.fieldSpeeds.vy,
            'omega': s.fieldSpeeds.omega,
            'moduleStates': [
              for (SwerveModuleTrajState ms in s.moduleStates)
                {
                  'fieldAngle': ms.fieldAngle.radians,
                  'fieldPosX': ms.fieldPos.x,
                  'fieldPosY': ms.fieldPos.y,
                  'speed': ms.speedMetersPerSecond,
                  'angle': ms.angle.radians,
                },
            ],
          },
      ],
    };
  }

  /// Deserialize a ghost auto from JSON.
  factory GhostAuto.fromJson(Map<String, dynamic> json) {
    List<TrajectoryState> states = [];
    for (Map<String, dynamic> s in json['states']) {
      TrajectoryState state = TrajectoryState.pregen(
        s['t'],
        ChassisSpeeds(
          vx: s['vx'],
          vy: s['vy'],
          omega: s['omega'],
        ),
        Pose2d(
          Translation2d(s['x'], s['y']),
          Rotation2d.fromRadians(s['rotation']),
        ),
      );

      // Restore module states
      if (s['moduleStates'] != null) {
        state.moduleStates = [
          for (Map<String, dynamic> ms in s['moduleStates'])
            _moduleStateFromJson(ms),
        ];
      }

      states.add(state);
    }

    List<Translation2d> moduleLocations = [];
    if (json['moduleLocations'] != null) {
      for (Map<String, dynamic> loc in json['moduleLocations']) {
        moduleLocations.add(Translation2d(loc['x'], loc['y']));
      }
    }

    return GhostAuto(
      name: json['name'] ?? 'Ghost Auto',
      trajectory: PathPlannerTrajectory.fromStates(states),
      bumperSize: Size(
        (json['bumperSize']?['width'] ?? 0.9).toDouble(),
        (json['bumperSize']?['height'] ?? 0.9).toDouble(),
      ),
      bumperOffset: Translation2d(
        json['bumperOffset']?['x'] ?? 0.0,
        json['bumperOffset']?['y'] ?? 0.0,
      ),
      moduleLocations: moduleLocations,
      holonomic: json['holonomic'] ?? true,
    );
  }

  static SwerveModuleTrajState _moduleStateFromJson(
      Map<String, dynamic> json) {
    SwerveModuleTrajState ms = SwerveModuleTrajState();
    ms.fieldAngle = Rotation2d.fromRadians(json['fieldAngle'] ?? 0.0);
    ms.fieldPos = Translation2d(
      json['fieldPosX'] ?? 0.0,
      json['fieldPosY'] ?? 0.0,
    );
    ms.speedMetersPerSecond = json['speed'] ?? 0.0;
    ms.angle = Rotation2d.fromRadians(json['angle'] ?? 0.0);
    return ms;
  }

  /// Export the ghost auto to a file chosen by the user.
  /// Returns true if the file was successfully saved.
  static Future<bool> exportToFile(GhostAuto ghostAuto) async {
    try {
      final saveLocation = await getSaveLocation(
        acceptedTypeGroups: [
          const XTypeGroup(
            label: 'Ghost Auto',
            extensions: ['ghostauto'],
          ),
        ],
        suggestedName: '${ghostAuto.name}.ghostauto',
      );

      if (saveLocation != null) {
        const JsonEncoder encoder = JsonEncoder.withIndent('  ');
        final file = File(saveLocation.path);
        await file.writeAsString(encoder.convert(ghostAuto.toJson()));
        Log.debug('Exported ghost auto: ${ghostAuto.name}');
        return true;
      }
    } catch (ex, stack) {
      Log.error('Failed to export ghost auto', ex, stack);
    }
    return false;
  }

  /// Import a ghost auto from a file chosen by the user.
  /// Returns null if the user cancelled or the file was invalid.
  static Future<GhostAuto?> importFromFile() async {
    try {
      final file = await openFile(
        acceptedTypeGroups: [
          const XTypeGroup(
            label: 'Ghost Auto',
            extensions: ['ghostauto'],
          ),
        ],
      );

      if (file != null) {
        final jsonStr = await file.readAsString();
        final json = jsonDecode(jsonStr);
        final ghostAuto = GhostAuto.fromJson(json);
        Log.debug('Imported ghost auto: ${ghostAuto.name}');
        return ghostAuto;
      }
    } catch (ex, stack) {
      Log.error('Failed to import ghost auto', ex, stack);
    }
    return null;
  }

  /// Get the total time of this ghost auto's trajectory.
  num getTotalTimeSeconds() {
    if (trajectory.states.isEmpty) return 0;
    return trajectory.states.last.timeSeconds;
  }

  /// Sample the ghost trajectory at a given time using simple linear pose
  /// interpolation. This avoids the velocity-integration drift that occurs
  /// in [TrajectoryState.interpolate] when heading changes near waypoints.
  TrajectoryState sampleLinear(num time) {
    final states = trajectory.states;
    if (states.isEmpty) return TrajectoryState();
    if (time <= states.first.timeSeconds) return states.first;
    if (time >= states.last.timeSeconds) return states.last;

    // Binary search for the bounding states
    int low = 1;
    int high = states.length - 1;
    while (low != high) {
      int mid = ((low + high) / 2).floor();
      if (states[mid].timeSeconds < time) {
        low = mid + 1;
      } else {
        high = mid;
      }
    }

    TrajectoryState endState = states[low];
    TrajectoryState startState = states[low - 1];

    num dt = endState.timeSeconds - startState.timeSeconds;
    if (dt.abs() < 1e-6) return endState;

    num t = (time - startState.timeSeconds) / dt;

    // Simple linear interpolation of pose (no velocity integration)
    TrajectoryState result = TrajectoryState();
    result.timeSeconds = time;
    result.pose = startState.pose.interpolate(endState.pose, t);
    result.heading = startState.heading;
    result.fieldSpeeds = ChassisSpeeds(
      vx: _lerp(startState.fieldSpeeds.vx, endState.fieldSpeeds.vx, t),
      vy: _lerp(startState.fieldSpeeds.vy, endState.fieldSpeeds.vy, t),
      omega: _lerp(startState.fieldSpeeds.omega, endState.fieldSpeeds.omega, t),
    );

    // Linearly interpolate module states
    for (int i = 0; i < startState.moduleStates.length; i++) {
      if (i < endState.moduleStates.length) {
        result.moduleStates
            .add(startState.moduleStates[i].interpolate(endState.moduleStates[i], t));
      }
    }

    return result;
  }

  static num _lerp(num a, num b, num t) => a + (b - a) * t;

  /// Export a ghost auto directly to a specific file path (no file picker).
  static Future<bool> exportToPath(GhostAuto ghostAuto, String filePath) async {
    try {
      const JsonEncoder encoder = JsonEncoder.withIndent('  ');
      final file = File(filePath);
      await file.writeAsString(encoder.convert(ghostAuto.toJson()));
      Log.debug('Exported ghost auto to: $filePath');
      return true;
    } catch (ex, stack) {
      Log.error('Failed to export ghost auto to path', ex, stack);
    }
    return false;
  }
}
