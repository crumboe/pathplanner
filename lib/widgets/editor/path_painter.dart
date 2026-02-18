import 'dart:convert';
import 'dart:math';
import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:pathplanner/path/choreo_path.dart';
import 'package:pathplanner/path/point_towards_zone.dart';
import 'package:pathplanner/path/rotation_target.dart';
import 'package:pathplanner/robot_features/feature.dart';
import 'package:pathplanner/trajectory/config.dart';
import 'package:pathplanner/trajectory/trajectory.dart';
import 'package:pathplanner/path/pathplanner_path.dart';
import 'package:pathplanner/path/waypoint.dart';
import 'package:pathplanner/util/prefs.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';
import 'package:pathplanner/widgets/field_image.dart';
import 'package:pathplanner/auto/ghost_auto.dart';
import 'package:pathplanner/util/path_painter_util.dart';
import 'package:shared_preferences/shared_preferences.dart';

class PathPainter extends CustomPainter {
  final ColorScheme colorScheme;
  final List<PathPlannerPath> paths;
  final List<ChoreoPath> choreoPaths;
  final FieldImage fieldImage;
  final bool simple;
  final bool hideOtherPathsOnHover;
  final String? hoveredPath;
  final int? hoveredWaypoint;
  final int? selectedWaypoint;
  final int? hoveredZone;
  final int? selectedZone;
  final int? hoveredPointZone;
  final int? selectedPointZone;
  final int? hoveredRotTarget;
  final int? selectedRotTarget;
  final int? hoveredMarker;
  final int? selectedMarker;
  final PathPlannerTrajectory? simulatedPath;
  final SharedPreferences prefs;
  final PathPlannerPath? optimizedPath;
  final List<GhostAuto> ghostAutos;
  final num ghostTimeOffset;

  late final RobotConfig robotConfig;
  late final num robotRadius;
  Animation<num>? previewTime;
  final List<Feature> robotFeatures = [];

  static double scale = 1;

  PathPainter({
    required this.colorScheme,
    required this.paths,
    this.choreoPaths = const [],
    required this.fieldImage,
    this.simple = false,
    this.hideOtherPathsOnHover = false,
    this.hoveredPath,
    this.hoveredWaypoint,
    this.selectedWaypoint,
    this.hoveredZone,
    this.selectedZone,
    this.hoveredPointZone,
    this.selectedPointZone,
    this.hoveredRotTarget,
    this.selectedRotTarget,
    this.hoveredMarker,
    this.selectedMarker,
    this.simulatedPath,
    Animation<double>? animation,
    required this.prefs,
    this.optimizedPath,
    this.ghostAutos = const [],
    this.ghostTimeOffset = 0,
  }) : super(repaint: animation) {
    robotConfig = RobotConfig.fromPrefs(prefs);
    robotRadius = sqrt((robotConfig.bumperSize.width *
                robotConfig.bumperSize.width) +
            (robotConfig.bumperSize.height * robotConfig.bumperSize.height)) /
        2.0;

    for (String featureJson in prefs.getStringList(PrefsKeys.robotFeatures) ??
        Defaults.robotFeatures) {
      try {
        robotFeatures.add(Feature.fromJson(jsonDecode(featureJson))!);
      } catch (_) {
        // Ignore and skip loading this feature
      }
    }

    if (simulatedPath != null && animation != null) {
      previewTime =
          Tween<num>(begin: 0, end: simulatedPath!.states.last.timeSeconds)
              .animate(animation);
    }

    // Ghost preview time is synced to the same wall-clock time as the main
    // preview. We reuse previewTime (which maps to real seconds) and clamp
    // to the ghost trajectory duration inside _paintGhostAuto.
    // No separate ghostPreviewTime tween needed.
  }

  @override
  void paint(Canvas canvas, Size size) {
    scale = size.width / fieldImage.defaultSize.width;

    _paintGrid(
        canvas, size, prefs.getBool(PrefsKeys.showGrid) ?? Defaults.showGrid);

    for (int i = 0; i < paths.length; i++) {
      if (hideOtherPathsOnHover &&
          hoveredPath != null &&
          hoveredPath != paths[i].name) {
        continue;
      }

      if (!simple) {
        _paintRadius(paths[i], canvas, scale);
      }

      _paintPathPoints(
          paths[i],
          canvas,
          (hoveredPath == paths[i].name)
              ? Colors.orange
              : colorScheme.secondary);

      if (robotConfig.holonomic) {
        _paintRotations(paths[i], canvas, scale);
      }

      _paintMarkers(paths[i], canvas);

      if (!simple) {
        for (int w = 0; w < paths[i].waypoints.length; w++) {
          _paintWaypoint(paths[i], canvas, scale, w);
        }
      } else {
        _paintWaypoint(paths[i], canvas, scale, 0);
        _paintWaypoint(paths[i], canvas, scale, paths[i].waypoints.length - 1);
      }

      _paintPointZonePositions(paths[i], canvas, scale);
    }

    for (int i = 0; i < choreoPaths.length; i++) {
      if (hideOtherPathsOnHover &&
          hoveredPath != null &&
          hoveredPath != choreoPaths[i].name) {
        continue;
      }

      if (choreoPaths[i].trajectory.states.isEmpty) {
        continue;
      }

      _paintTrajectory(
          choreoPaths[i].trajectory,
          canvas,
          (hoveredPath == choreoPaths[i].name)
              ? Colors.orange
              : colorScheme.secondary);
      _paintChoreoWaypoint(
          choreoPaths[i].trajectory.states.first, canvas, Colors.green, scale);
      _paintChoreoWaypoint(
          choreoPaths[i].trajectory.states.last, canvas, Colors.red, scale);
      _paintChoreoMarkers(choreoPaths[i], canvas);
    }

    if (optimizedPath != null) {
      _paintPathPoints(optimizedPath!, canvas, Colors.deepPurpleAccent, 4.0);
    }

    for (int i = 1; i < paths.length; i++) {
      // Paint warnings between breaks in paths
      Translation2d prevPathEnd = paths[i - 1].pathPoints.last.position;
      Translation2d pathStart = paths[i].pathPoints.first.position;

      if (prevPathEnd.getDistance(pathStart) >= 0.25) {
        _paintBreakWarning(prevPathEnd, pathStart, canvas, scale);
      }
    }

    for (int i = 1; i < choreoPaths.length; i++) {
      // Paint warnings between breaks in paths
      Translation2d prevPathEnd =
          choreoPaths[i - 1].trajectory.states.last.pose.translation;
      Translation2d pathStart =
          choreoPaths[i].trajectory.states.first.pose.translation;

      if (prevPathEnd.getDistance(pathStart) >= 0.25) {
        _paintBreakWarning(prevPathEnd, pathStart, canvas, scale);
      }
    }

    if (prefs.getBool(PrefsKeys.showStates) ?? Defaults.showStates) {
      _paintTrajectoryStates(simulatedPath, canvas);
    }

    // Paint ghost autos (reference autos from other robots) behind the main preview
    _paintGhostAutos(canvas, size);

    // Paint collision warnings between ghosts and main trajectory
    _paintGhostCollisions(canvas);

    if (previewTime != null) {
      TrajectoryState state = simulatedPath!.sample(previewTime!.value);
      Rotation2d rotation = state.pose.rotation;

      if (robotConfig.holonomic && state.moduleStates.isNotEmpty) {
        // Calculate the module positions based off of the robot position
        // so they don't move relative to the robot when interpolating
        // between trajectory states
        List<Pose2d> modPoses = [
          Pose2d(
              state.pose.translation +
                  robotConfig.moduleLocations[0].rotateBy(rotation),
              state.moduleStates[0].fieldAngle),
          Pose2d(
              state.pose.translation +
                  robotConfig.moduleLocations[1].rotateBy(rotation),
              state.moduleStates[1].fieldAngle),
          Pose2d(
              state.pose.translation +
                  robotConfig.moduleLocations[2].rotateBy(rotation),
              state.moduleStates[2].fieldAngle),
          Pose2d(
              state.pose.translation +
                  robotConfig.moduleLocations[3].rotateBy(rotation),
              state.moduleStates[3].fieldAngle),
        ];
        PathPainterUtil.paintRobotModules(
            modPoses, fieldImage, scale, canvas, colorScheme.primary);
      }

      PathPainterUtil.paintRobotOutline(
        Pose2d(state.pose.translation, rotation),
        fieldImage,
        robotConfig.bumperSize,
        robotConfig.bumperOffset,
        scale,
        canvas,
        colorScheme.primary,
        colorScheme.surfaceContainer,
        robotFeatures,
        showDetails: prefs.getBool(PrefsKeys.showRobotDetails) ??
            Defaults.showRobotDetails,
        bumperStrokeWidth: 4.0,
      );
    }
  }

  @override
  bool shouldRepaint(PathPainter oldDelegate) {
    return true; // This will just be repainted all the time anyways from the animation
  }

  void _paintTrajectoryStates(PathPlannerTrajectory? traj, Canvas canvas) {
    if (traj == null) {
      return;
    }

    var paint = Paint()..style = PaintingStyle.fill;

    num maxVel = 0.0;
    for (TrajectoryState s in traj.states) {
      maxVel = max(
          maxVel, sqrt(pow(s.fieldSpeeds.vx, 2) + pow(s.fieldSpeeds.vy, 2)));
    }

    for (TrajectoryState s in traj.states) {
      num normalizedVel =
          sqrt(pow(s.fieldSpeeds.vx, 2) + pow(s.fieldSpeeds.vy, 2)) / maxVel;
      normalizedVel = normalizedVel.clamp(0.0, 1.0);

      if (normalizedVel <= 0.33) {
        // Lerp between red and orange
        paint.color =
            Color.lerp(Colors.red, Colors.orange, normalizedVel / 0.33)!;
      } else if (normalizedVel <= 0.67) {
        // Lerp between orange and yellow
        paint.color = Color.lerp(
            Colors.orange, Colors.yellow, (normalizedVel - 0.33) / 0.34)!;
      } else {
        // Lerp between yellow and green
        paint.color = Color.lerp(
            Colors.yellow, Colors.green, (normalizedVel - 0.67) / 0.33)!;
      }
      Offset pos = PathPainterUtil.pointToPixelOffset(
          s.pose.translation, scale, fieldImage);
      canvas.drawCircle(pos, 3.0, paint);
    }
  }

  void _paintTrajectory(
      PathPlannerTrajectory traj, Canvas canvas, Color baseColor) {
    var paint = Paint()
      ..style = PaintingStyle.stroke
      ..color = baseColor
      ..strokeWidth = 2;

    Path p = Path();

    Offset start = PathPainterUtil.pointToPixelOffset(
        traj.states.first.pose.translation, scale, fieldImage);
    p.moveTo(start.dx, start.dy);

    for (int i = 1; i < traj.states.length; i++) {
      Offset pos = PathPainterUtil.pointToPixelOffset(
          traj.states[i].pose.translation, scale, fieldImage);

      p.lineTo(pos.dx, pos.dy);
    }

    canvas.drawPath(p, paint);
  }

  /// Paint the ghost auto trajectory line and ghost robot at the current preview time.
  void _paintGhostAutos(Canvas canvas, Size size) {
    for (int gi = 0; gi < ghostAutos.length; gi++) {
      final ghost = ghostAutos[gi];
      if (ghost.trajectory.states.isEmpty) continue;

      Color ghostColor = GhostAuto.ghostColors[gi % GhostAuto.ghostColors.length];
      Color ghostPathColor = GhostAuto.ghostPathColors[gi % GhostAuto.ghostPathColors.length];
      const Color ghostOutlineColor = Color(0x66000000);

      // Draw the ghost trajectory line
      var pathPaint = Paint()
        ..style = PaintingStyle.stroke
        ..color = ghostPathColor
        ..strokeWidth = 2;

      Path p = Path();
      Offset start = PathPainterUtil.pointToPixelOffset(
          ghost.trajectory.states.first.pose.translation,
          scale,
          fieldImage);
      p.moveTo(start.dx, start.dy);

      for (int i = 1; i < ghost.trajectory.states.length; i++) {
        Offset pos = PathPainterUtil.pointToPixelOffset(
            ghost.trajectory.states[i].pose.translation,
            scale,
            fieldImage);
        p.lineTo(pos.dx, pos.dy);
      }
      canvas.drawPath(p, pathPaint);

      // Draw ghost start and end indicators
      _paintGhostEndpoint(
          ghost.trajectory.states.first, canvas, ghostColor.withOpacity(0.6));
      _paintGhostEndpoint(
          ghost.trajectory.states.last, canvas, ghostColor.withOpacity(0.4));

      // Draw ghost robot at current time (synced to main preview wall-clock time)
      if (previewTime != null) {
        num ghostTime = previewTime!.value + ghostTimeOffset;

        // Clamp to ghost trajectory duration
        num ghostTotalTime = ghost.getTotalTimeSeconds();
        if (ghostTime > ghostTotalTime) {
          ghostTime = ghostTotalTime;
        }

        // Use sampleLinear to avoid velocity-integration drift at waypoints
        TrajectoryState ghostState = ghost.sampleLinear(ghostTime);
        Rotation2d ghostRotation = ghostState.pose.rotation;

        // Draw ghost swerve modules
        if (ghost.holonomic &&
            ghostState.moduleStates.isNotEmpty &&
            ghost.moduleLocations.length == ghostState.moduleStates.length) {
          List<Pose2d> ghostModPoses = [];
          for (int i = 0; i < ghost.moduleLocations.length; i++) {
            ghostModPoses.add(Pose2d(
              ghostState.pose.translation +
                  ghost.moduleLocations[i].rotateBy(ghostRotation),
              ghostState.moduleStates[i].fieldAngle,
            ));
          }
          PathPainterUtil.paintRobotModules(
              ghostModPoses, fieldImage, scale, canvas, ghostColor);
        }

        // Draw ghost bumper outline
        PathPainterUtil.paintRobotOutline(
          Pose2d(ghostState.pose.translation, ghostRotation),
          fieldImage,
          ghost.bumperSize,
          ghost.bumperOffset,
          scale,
          canvas,
          ghostColor,
          ghostOutlineColor,
          [], // No features for ghost robot
        );
      }
    }
  }

  /// Paint a small circle endpoint indicator for the ghost trajectory.
  void _paintGhostEndpoint(
      TrajectoryState state, Canvas canvas, Color color) {
    var paint = Paint()
      ..style = PaintingStyle.fill
      ..color = color;
    canvas.drawCircle(
        PathPainterUtil.pointToPixelOffset(
            state.pose.translation, scale, fieldImage),
        PathPainterUtil.uiPointSizeToPixels(18, scale, fieldImage),
        paint);
  }

  /// Detect and paint collision warnings where two robots overlap in both
  /// position and time. Checks main trajectory vs each ghost, and each ghost
  /// pair. Uses bumper bounding-circle approximation for speed.
  void _paintGhostCollisions(Canvas canvas) {
    if (ghostAutos.isEmpty) return;

    // Build a list of all trajectories to compare (main + ghosts)
    // Each entry: (trajectory, bumperRadius, timeOffset, colorIndex)
    final List<_CollisionEntry> entries = [];

    // Main robot trajectory
    if (simulatedPath != null && simulatedPath!.states.isNotEmpty) {
      num mainRadius = sqrt(
              robotConfig.bumperSize.width * robotConfig.bumperSize.width +
                  robotConfig.bumperSize.height *
                      robotConfig.bumperSize.height) /
          2.0;
      entries.add(_CollisionEntry(
        trajectory: simulatedPath!,
        bumperRadius: mainRadius,
        timeOffset: 0,
        colorIndex: -1, // main robot
        sampleFn: (num t) => simulatedPath!.sample(t),
      ));
    }

    // Ghost trajectories
    for (int gi = 0; gi < ghostAutos.length; gi++) {
      final ghost = ghostAutos[gi];
      if (ghost.trajectory.states.isEmpty) continue;
      num ghostRadius = sqrt(
              ghost.bumperSize.width * ghost.bumperSize.width +
                  ghost.bumperSize.height * ghost.bumperSize.height) /
          2.0;
      entries.add(_CollisionEntry(
        trajectory: ghost.trajectory,
        bumperRadius: ghostRadius,
        timeOffset: ghostTimeOffset,
        colorIndex: gi,
        sampleFn: (num t) => ghost.sampleLinear(t),
      ));
    }

    if (entries.length < 2) return;

    // Collision warning paint
    const Color warningColor = Color(0xFFFF0000);
    final warningPaint = Paint()
      ..style = PaintingStyle.stroke
      ..color = warningColor
      ..strokeWidth = 2.5;
    final warningFillPaint = Paint()
      ..style = PaintingStyle.fill
      ..color = const Color(0x44FF0000);
    final warningIconPaint = Paint()
      ..style = PaintingStyle.fill
      ..color = warningColor;

    // Check every pair of entries for time-position overlap.
    // Sample at 0.1s intervals for performance.
    const num sampleDt = 0.1;
    final Set<String> paintedPositions = {}; // avoid duplicate markers

    for (int a = 0; a < entries.length; a++) {
      for (int b = a + 1; b < entries.length; b++) {
        final ea = entries[a];
        final eb = entries[b];

        // Find overlapping time range
        num aStart = ea.trajectory.states.first.timeSeconds + ea.timeOffset;
        num aEnd = ea.trajectory.states.last.timeSeconds + ea.timeOffset;
        num bStart = eb.trajectory.states.first.timeSeconds + eb.timeOffset;
        num bEnd = eb.trajectory.states.last.timeSeconds + eb.timeOffset;

        num overlapStart = aStart > bStart ? aStart : bStart;
        num overlapEnd = aEnd < bEnd ? aEnd : bEnd;

        if (overlapStart >= overlapEnd) continue; // No time overlap

        num collisionThreshold = ea.bumperRadius + eb.bumperRadius;
        bool inCollision = false;

        for (num t = overlapStart; t <= overlapEnd; t += sampleDt) {
          // Sample both trajectories at time t (adjusting for offsets)
          TrajectoryState stateA = ea.sampleFn(t - ea.timeOffset);
          TrajectoryState stateB = eb.sampleFn(t - eb.timeOffset);

          num dist = stateA.pose.translation
              .getDistance(stateB.pose.translation);

          if (dist < collisionThreshold) {
            if (!inCollision) {
              inCollision = true;
              // Paint collision marker at midpoint
              Translation2d midpoint = Translation2d(
                (stateA.pose.translation.x + stateB.pose.translation.x) / 2,
                (stateA.pose.translation.y + stateB.pose.translation.y) / 2,
              );
              String posKey =
                  '${(midpoint.x * 10).round()},${(midpoint.y * 10).round()}';
              if (!paintedPositions.contains(posKey)) {
                paintedPositions.add(posKey);
                Offset center = PathPainterUtil.pointToPixelOffset(
                    midpoint, scale, fieldImage);
                double markerRadius =
                    PathPainterUtil.uiPointSizeToPixels(25, scale, fieldImage);

                // Red translucent circle
                canvas.drawCircle(center, markerRadius, warningFillPaint);
                // Red outline
                canvas.drawCircle(center, markerRadius, warningPaint);
                // Warning exclamation mark
                _paintWarningIcon(canvas, center, markerRadius * 0.6,
                    warningIconPaint);
              }
            }
          } else {
            inCollision = false;
          }
        }
      }
    }
  }

  /// Paint a simple exclamation mark (!) warning icon.
  void _paintWarningIcon(
      Canvas canvas, Offset center, double size, Paint paint) {
    // Exclamation line
    double lineTop = center.dy - size * 0.5;
    double lineBottom = center.dy + size * 0.15;
    double lineWidth = size * 0.22;
    canvas.drawRRect(
      RRect.fromRectAndRadius(
        Rect.fromLTWH(
            center.dx - lineWidth / 2, lineTop, lineWidth, lineBottom - lineTop),
        Radius.circular(lineWidth / 2),
      ),
      paint,
    );
    // Dot
    double dotY = center.dy + size * 0.4;
    canvas.drawCircle(Offset(center.dx, dotY), lineWidth * 0.6, paint);
  }


  void _paintChoreoWaypoint(
      TrajectoryState state, Canvas canvas, Color color, double scale) {
    var paint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2
      ..color = color;

    // draw anchor point
    paint.style = PaintingStyle.fill;
    canvas.drawCircle(
        PathPainterUtil.pointToPixelOffset(
            state.pose.translation, scale, fieldImage),
        PathPainterUtil.uiPointSizeToPixels(25, scale, fieldImage),
        paint);
    paint.style = PaintingStyle.stroke;
    paint.color = colorScheme.surfaceContainer;
    canvas.drawCircle(
        PathPainterUtil.pointToPixelOffset(
            state.pose.translation, scale, fieldImage),
        PathPainterUtil.uiPointSizeToPixels(25, scale, fieldImage),
        paint);

    // Draw robot
    PathPainterUtil.paintRobotOutline(
        state.pose,
        fieldImage,
        robotConfig.bumperSize,
        robotConfig.bumperOffset,
        scale,
        canvas,
        color.withAlpha(50),
        colorScheme.surfaceContainer.withAlpha(40),
        robotFeatures);
  }

  void _paintPathPoints(PathPlannerPath path, Canvas canvas, Color baseColor,
      [double strokeWidth = 2.0]) {
    var paint = Paint()
      ..style = PaintingStyle.stroke
      ..color = baseColor
      ..strokeWidth = strokeWidth;

    Path p = Path();

    Offset start = PathPainterUtil.pointToPixelOffset(
        path.pathPoints[0].position, scale, fieldImage);
    p.moveTo(start.dx, start.dy);

    for (int i = 1; i < path.pathPoints.length; i++) {
      Offset pos = PathPainterUtil.pointToPixelOffset(
          path.pathPoints[i].position, scale, fieldImage);

      p.lineTo(pos.dx, pos.dy);
    }

    canvas.drawPath(p, paint);

    if (selectedZone != null) {
      paint.color = Colors.orange;
      paint.strokeWidth = 6;
      p.reset();

      num startPos = path.constraintZones[selectedZone!].minWaypointRelativePos;
      num endPos = path.constraintZones[selectedZone!].maxWaypointRelativePos;

      Offset start = PathPainterUtil.pointToPixelOffset(
          path.samplePath(startPos), scale, fieldImage);
      p.moveTo(start.dx, start.dy);

      for (num t = startPos + 0.05; t <= endPos; t += 0.05) {
        Offset pos = PathPainterUtil.pointToPixelOffset(
            path.samplePath(t), scale, fieldImage);

        p.lineTo(pos.dx, pos.dy);
      }
      Offset end = PathPainterUtil.pointToPixelOffset(
          path.samplePath(endPos), scale, fieldImage);
      p.lineTo(end.dx, end.dy);

      canvas.drawPath(p, paint);
    }

    if (hoveredZone != null && selectedZone != hoveredZone) {
      paint.color = Colors.deepPurpleAccent;
      paint.strokeWidth = 6;
      p.reset();

      num startPos = path.constraintZones[hoveredZone!].minWaypointRelativePos;
      num endPos = path.constraintZones[hoveredZone!].maxWaypointRelativePos;

      Offset start = PathPainterUtil.pointToPixelOffset(
          path.samplePath(startPos), scale, fieldImage);
      p.moveTo(start.dx, start.dy);

      for (num t = startPos + 0.05; t <= endPos; t += 0.05) {
        Offset pos = PathPainterUtil.pointToPixelOffset(
            path.samplePath(t), scale, fieldImage);

        p.lineTo(pos.dx, pos.dy);
      }
      Offset end = PathPainterUtil.pointToPixelOffset(
          path.samplePath(endPos), scale, fieldImage);
      p.lineTo(end.dx, end.dy);

      canvas.drawPath(p, paint);
    }

    if (selectedPointZone != null) {
      paint.color = Colors.orange;
      paint.strokeWidth = 6;
      p.reset();

      num startPos =
          path.pointTowardsZones[selectedPointZone!].minWaypointRelativePos;
      num endPos =
          path.pointTowardsZones[selectedPointZone!].maxWaypointRelativePos;

      Offset start = PathPainterUtil.pointToPixelOffset(
          path.samplePath(startPos), scale, fieldImage);
      p.moveTo(start.dx, start.dy);

      for (num t = startPos + 0.05; t <= endPos; t += 0.05) {
        Offset pos = PathPainterUtil.pointToPixelOffset(
            path.samplePath(t), scale, fieldImage);

        p.lineTo(pos.dx, pos.dy);
      }

      Offset end = PathPainterUtil.pointToPixelOffset(
          path.samplePath(endPos), scale, fieldImage);
      p.lineTo(end.dx, end.dy);

      canvas.drawPath(p, paint);
    }

    if (hoveredPointZone != null && selectedPointZone != hoveredPointZone) {
      paint.color = Colors.deepPurpleAccent;
      paint.strokeWidth = 6;
      p.reset();

      num startPos =
          path.pointTowardsZones[hoveredPointZone!].minWaypointRelativePos;
      num endPos =
          path.pointTowardsZones[hoveredPointZone!].maxWaypointRelativePos;

      Offset start = PathPainterUtil.pointToPixelOffset(
          path.samplePath(startPos), scale, fieldImage);
      p.moveTo(start.dx, start.dy);

      for (num t = startPos + 0.05; t <= endPos; t += 0.05) {
        Offset pos = PathPainterUtil.pointToPixelOffset(
            path.samplePath(t), scale, fieldImage);

        p.lineTo(pos.dx, pos.dy);
      }

      Offset end = PathPainterUtil.pointToPixelOffset(
          path.samplePath(endPos), scale, fieldImage);
      p.lineTo(end.dx, end.dy);

      canvas.drawPath(p, paint);
    }

    if (selectedMarker != null && path.eventMarkers[selectedMarker!].isZoned) {
      paint.color = Colors.orange;
      paint.strokeWidth = 6;
      p.reset();

      num startPos = path.eventMarkers[selectedMarker!].waypointRelativePos;
      num endPos = path.eventMarkers[selectedMarker!].endWaypointRelativePos!;

      Offset start = PathPainterUtil.pointToPixelOffset(
          path.samplePath(startPos), scale, fieldImage);
      p.moveTo(start.dx, start.dy);

      for (num t = startPos + 0.05; t <= endPos; t += 0.05) {
        Offset pos = PathPainterUtil.pointToPixelOffset(
            path.samplePath(t), scale, fieldImage);

        p.lineTo(pos.dx, pos.dy);
      }
      Offset end = PathPainterUtil.pointToPixelOffset(
          path.samplePath(endPos), scale, fieldImage);
      p.lineTo(end.dx, end.dy);

      canvas.drawPath(p, paint);
    }

    if (hoveredMarker != null &&
        hoveredMarker != selectedMarker &&
        path.eventMarkers[hoveredMarker!].isZoned) {
      paint.color = Colors.deepPurpleAccent;
      paint.strokeWidth = 6;
      p.reset();

      num startPos = path.eventMarkers[hoveredMarker!].waypointRelativePos;
      num endPos = path.eventMarkers[hoveredMarker!].endWaypointRelativePos!;

      Offset start = PathPainterUtil.pointToPixelOffset(
          path.samplePath(startPos), scale, fieldImage);
      p.moveTo(start.dx, start.dy);

      for (num t = startPos + 0.05; t <= endPos; t += 0.05) {
        Offset pos = PathPainterUtil.pointToPixelOffset(
            path.samplePath(t), scale, fieldImage);

        p.lineTo(pos.dx, pos.dy);
      }
      Offset end = PathPainterUtil.pointToPixelOffset(
          path.samplePath(endPos), scale, fieldImage);
      p.lineTo(end.dx, end.dy);

      canvas.drawPath(p, paint);
    }
  }

  void _paintMarkers(PathPlannerPath path, Canvas canvas) {
    for (int i = 0; i < path.eventMarkers.length; i++) {
      var position = path.samplePath(path.eventMarkers[i].waypointRelativePos);

      Color markerColor = Colors.grey[700]!;
      Color markerStrokeColor = colorScheme.surfaceContainer;
      if (selectedMarker == i) {
        markerColor = Colors.orange;
      } else if (hoveredMarker == i) {
        markerColor = Colors.deepPurpleAccent;
      }

      Offset markerPos =
          PathPainterUtil.pointToPixelOffset(position, scale, fieldImage);

      PathPainterUtil.paintMarker(
          canvas, markerPos, markerColor, markerStrokeColor);
    }
  }

  void _paintChoreoMarkers(ChoreoPath path, Canvas canvas) {
    for (num timestamp in path.eventMarkerTimes) {
      TrajectoryState s = path.trajectory.sample(timestamp);
      Offset markerPos = PathPainterUtil.pointToPixelOffset(
          s.pose.translation, scale, fieldImage);

      PathPainterUtil.paintMarker(
          canvas, markerPos, Colors.grey[700]!, colorScheme.onSurface);
    }
  }

  void _paintPointZonePositions(
      PathPlannerPath path, Canvas canvas, double scale) {
    if (selectedPointZone != null) {
      final paint = Paint()
        ..style = PaintingStyle.fill
        ..color = Colors.orange
        ..strokeWidth = 3;

      PointTowardsZone z = path.pointTowardsZones[selectedPointZone!];
      final location = PathPainterUtil.pointToPixelOffset(
          z.fieldPosition, scale, fieldImage);

      canvas.drawCircle(location,
          PathPainterUtil.uiPointSizeToPixels(25, scale, fieldImage), paint);

      paint.style = PaintingStyle.stroke;
      canvas.drawCircle(location,
          PathPainterUtil.uiPointSizeToPixels(40, scale, fieldImage), paint);
    }

    if (hoveredPointZone != null && hoveredPointZone != selectedPointZone) {
      final paint = Paint()
        ..style = PaintingStyle.fill
        ..color = Colors.deepPurpleAccent
        ..strokeWidth = 3;

      PointTowardsZone z = path.pointTowardsZones[hoveredPointZone!];
      final location = PathPainterUtil.pointToPixelOffset(
          z.fieldPosition, scale, fieldImage);

      canvas.drawCircle(location,
          PathPainterUtil.uiPointSizeToPixels(25, scale, fieldImage), paint);

      paint.style = PaintingStyle.stroke;
      canvas.drawCircle(location,
          PathPainterUtil.uiPointSizeToPixels(40, scale, fieldImage), paint);
    }
  }

  void _paintRotations(PathPlannerPath path, Canvas canvas, double scale) {
    for (int i = 0; i < path.pathPoints.length - 1; i++) {
      if (path.pathPoints[i].rotationTarget != null &&
          path.pathPoints[i].rotationTarget!.displayInEditor) {
        RotationTarget target = path.pathPoints[i].rotationTarget!;
        Color rotationColor = Colors.grey[700]!.withAlpha(60);
        if (selectedRotTarget != null &&
            path.rotationTargets[selectedRotTarget!] == target) {
          rotationColor = Colors.orange.withAlpha(80);
        } else if (hoveredRotTarget != null &&
            path.rotationTargets[hoveredRotTarget!] == target) {
          rotationColor = Colors.deepPurpleAccent.withAlpha(80);
        }

        PathPainterUtil.paintRobotOutline(
            Pose2d(path.pathPoints[i].position, target.rotation),
            fieldImage,
            robotConfig.bumperSize,
            robotConfig.bumperOffset,
            scale,
            canvas,
            rotationColor,
            colorScheme.surfaceContainer.withAlpha(40),
            robotFeatures);
      }
    }

    PathPainterUtil.paintRobotOutline(
        Pose2d(path.waypoints.first.anchor, path.idealStartingState.rotation),
        fieldImage,
        robotConfig.bumperSize,
        robotConfig.bumperOffset,
        scale,
        canvas,
        Colors.green.withAlpha(50),
        colorScheme.surfaceContainer.withAlpha(40),
        robotFeatures);

    PathPainterUtil.paintRobotOutline(
        Pose2d(path.waypoints[path.waypoints.length - 1].anchor,
            path.goalEndState.rotation),
        fieldImage,
        robotConfig.bumperSize,
        robotConfig.bumperOffset,
        scale,
        canvas,
        Colors.red.withAlpha(50),
        colorScheme.surfaceContainer.withAlpha(40),
        robotFeatures);
  }

  void _paintBreakWarning(Translation2d prevPathEnd, Translation2d pathStart,
      Canvas canvas, double scale) {
    var paint = Paint()
      ..style = PaintingStyle.stroke
      ..color = Colors.yellow[800]!
      ..strokeWidth = 3;

    final p1 =
        PathPainterUtil.pointToPixelOffset(prevPathEnd, scale, fieldImage);
    final p2 = PathPainterUtil.pointToPixelOffset(pathStart, scale, fieldImage);
    final distance = (p2 - p1).distance;
    final normalizedPattern = [7, 5].map((width) => width / distance).toList();
    final points = <Offset>[];
    double t = 0.0;
    int i = 0;
    while (t < 1.0) {
      points.add(Offset.lerp(p1, p2, t)!);
      t += normalizedPattern[i++];
      points.add(Offset.lerp(p1, p2, t.clamp(0.0, 1.0))!);
      t += normalizedPattern[i++];
      i %= normalizedPattern.length;
    }
    canvas.drawPoints(PointMode.lines, points, paint);

    Offset middle = Offset.lerp(p1, p2, 0.5)!;

    const IconData warningIcon = Icons.warning_rounded;

    TextPainter textPainter = TextPainter(
      textDirection: TextDirection.ltr,
      text: TextSpan(
        text: String.fromCharCode(warningIcon.codePoint),
        style: TextStyle(
          fontSize: 40,
          color: Colors.yellow[700]!,
          fontFamily: warningIcon.fontFamily,
        ),
      ),
    );

    TextPainter textStrokePainter = TextPainter(
      textDirection: TextDirection.ltr,
      text: TextSpan(
        text: String.fromCharCode(warningIcon.codePoint),
        style: TextStyle(
          fontSize: 40,
          fontFamily: warningIcon.fontFamily,
          foreground: Paint()
            ..style = PaintingStyle.stroke
            ..strokeWidth = 0.5
            ..color = colorScheme.surfaceContainer,
        ),
      ),
    );

    textPainter.layout();
    textStrokePainter.layout();

    textPainter.paint(canvas, middle - const Offset(20, 25));
    textStrokePainter.paint(canvas, middle - const Offset(20, 25));
  }

  void _paintRadius(PathPlannerPath path, Canvas canvas, double scale) {
    if (selectedWaypoint != null) {
      var paint = Paint()
        ..style = PaintingStyle.stroke
        ..color = colorScheme.surfaceContainerHighest
        ..strokeWidth = 2;

      canvas.drawCircle(
          PathPainterUtil.pointToPixelOffset(
              path.waypoints[selectedWaypoint!].anchor, scale, fieldImage),
          PathPainterUtil.metersToPixels(
              robotRadius.toDouble(), scale, fieldImage),
          paint);
    }
  }

  void _paintWaypoint(
      PathPlannerPath path, Canvas canvas, double scale, int waypointIdx) {
    var paint = Paint()
      ..style = PaintingStyle.stroke
      ..strokeWidth = 2;

    if (waypointIdx == selectedWaypoint) {
      paint.color = Colors.orange;
    } else if (waypointIdx == hoveredWaypoint) {
      paint.color = Colors.deepPurpleAccent;
    } else {
      paint.color = Colors.grey[700]!;
    }

    Waypoint waypoint = path.waypoints[waypointIdx];

    if (!simple) {
      //draw control point lines
      if (waypoint.nextControl != null) {
        canvas.drawLine(
            PathPainterUtil.pointToPixelOffset(
                waypoint.anchor, scale, fieldImage),
            PathPainterUtil.pointToPixelOffset(
                waypoint.nextControl!, scale, fieldImage),
            paint);
      }
      if (waypoint.prevControl != null) {
        canvas.drawLine(
            PathPainterUtil.pointToPixelOffset(
                waypoint.anchor, scale, fieldImage),
            PathPainterUtil.pointToPixelOffset(
                waypoint.prevControl!, scale, fieldImage),
            paint);
      }
    }

    if (waypointIdx == 0) {
      paint.color = Colors.green;
    } else if (waypointIdx == path.waypoints.length - 1) {
      paint.color = Colors.red;
    } else {
      paint.color = colorScheme.secondary;
    }

    if (waypointIdx == selectedWaypoint) {
      paint.color = Colors.orange;
    } else if (waypointIdx == hoveredWaypoint) {
      paint.color = Colors.deepPurpleAccent;
    }

    // draw anchor point
    paint.style = PaintingStyle.fill;
    canvas.drawCircle(
        PathPainterUtil.pointToPixelOffset(waypoint.anchor, scale, fieldImage),
        PathPainterUtil.uiPointSizeToPixels(25, scale, fieldImage),
        paint);
    paint.style = PaintingStyle.stroke;
    paint.color = colorScheme.surfaceContainer;
    canvas.drawCircle(
        PathPainterUtil.pointToPixelOffset(waypoint.anchor, scale, fieldImage),
        PathPainterUtil.uiPointSizeToPixels(25, scale, fieldImage),
        paint);

    if (!simple) {
      // draw control points
      if (waypoint.nextControl != null) {
        paint.style = PaintingStyle.fill;
        if (waypointIdx == selectedWaypoint) {
          paint.color = Colors.orange;
        } else if (waypointIdx == hoveredWaypoint) {
          paint.color = Colors.deepPurpleAccent;
        } else {
          paint.color = colorScheme.secondary;
        }

        canvas.drawCircle(
            PathPainterUtil.pointToPixelOffset(
                waypoint.nextControl!, scale, fieldImage),
            PathPainterUtil.uiPointSizeToPixels(20, scale, fieldImage),
            paint);
        paint.style = PaintingStyle.stroke;
        paint.color = colorScheme.surfaceContainer;
        canvas.drawCircle(
            PathPainterUtil.pointToPixelOffset(
                waypoint.nextControl!, scale, fieldImage),
            PathPainterUtil.uiPointSizeToPixels(20, scale, fieldImage),
            paint);
      }
      if (waypoint.prevControl != null) {
        paint.style = PaintingStyle.fill;
        if (waypointIdx == selectedWaypoint) {
          paint.color = Colors.orange;
        } else if (waypointIdx == hoveredWaypoint) {
          paint.color = Colors.deepPurpleAccent;
        } else {
          paint.color = colorScheme.secondary;
        }

        canvas.drawCircle(
            PathPainterUtil.pointToPixelOffset(
                waypoint.prevControl!, scale, fieldImage),
            PathPainterUtil.uiPointSizeToPixels(20, scale, fieldImage),
            paint);
        paint.style = PaintingStyle.stroke;
        paint.color = colorScheme.surfaceContainer;
        canvas.drawCircle(
            PathPainterUtil.pointToPixelOffset(
                waypoint.prevControl!, scale, fieldImage),
            PathPainterUtil.uiPointSizeToPixels(20, scale, fieldImage),
            paint);
      }
    }
  }

  void _paintGrid(Canvas canvas, Size size, bool showGrid) {
    if (!showGrid) return;

    final paint = Paint()
      ..color = colorScheme.secondary.withAlpha(50) // More transparent
      ..strokeWidth = 1;

    double gridSpacing = PathPainterUtil.metersToPixels(0.5, scale, fieldImage);

    for (double x = 0; x <= size.width; x += gridSpacing) {
      canvas.drawLine(Offset(x, 0), Offset(x, size.height), paint);
    }

    for (double y = 0; y <= size.height; y += gridSpacing) {
      canvas.drawLine(Offset(0, y), Offset(size.width, y), paint);
    }
  }
}

/// Helper class for collision detection between trajectories.
class _CollisionEntry {
  final PathPlannerTrajectory trajectory;
  final num bumperRadius;
  final num timeOffset;
  final int colorIndex; // -1 = main robot
  final TrajectoryState Function(num t) sampleFn;

  const _CollisionEntry({
    required this.trajectory,
    required this.bumperRadius,
    required this.timeOffset,
    required this.colorIndex,
    required this.sampleFn,
  });
}
