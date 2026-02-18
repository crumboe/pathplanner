import 'package:collection/collection.dart';
import 'package:flutter/material.dart';
import 'package:pathplanner/auto/ghost_auto.dart';
import 'package:pathplanner/auto/pathplanner_auto.dart';
import 'package:pathplanner/pages/path_editor_page.dart';
import 'package:pathplanner/path/choreo_path.dart';
import 'package:pathplanner/path/pathplanner_path.dart';
import 'package:pathplanner/path/waypoint.dart';
import 'package:pathplanner/services/ghost_sync_service.dart';
import 'package:pathplanner/services/pplib_telemetry.dart';
import 'package:pathplanner/util/wpimath/geometry.dart';
import 'package:pathplanner/widgets/conditional_widget.dart';
import 'package:pathplanner/widgets/custom_appbar.dart';
import 'package:pathplanner/widgets/editor/split_auto_editor.dart';
import 'package:pathplanner/widgets/field_image.dart';
import 'package:pathplanner/widgets/keyboard_shortcuts.dart';
import 'package:pathplanner/widgets/renamable_title.dart';
import 'package:shared_preferences/shared_preferences.dart';
import 'package:undo/undo.dart';

/// Result returned when the user chooses to edit a path from the auto editor.
/// Carries the ghost auto and time offset so the path editor can show the ghost
/// at the correct position in the auto timeline.
class EditPathResult {
  final String pathName;
  final List<GhostAuto> ghostAutos;
  final num ghostTimeOffset;

  const EditPathResult({
    required this.pathName,
    this.ghostAutos = const [],
    this.ghostTimeOffset = 0,
  });
}

class AutoEditorPage extends StatefulWidget {
  final SharedPreferences prefs;
  final PathPlannerAuto auto;
  final List<PathPlannerPath> allPaths;
  final List<ChoreoPath> allChoreoPaths;
  final List<String> allPathNames;
  final FieldImage fieldImage;
  final ValueChanged<String> onRenamed;
  final ChangeStack undoStack;
  final bool shortcuts;
  final PPLibTelemetry? telemetry;
  final bool hotReload;
  final GhostSyncService? ghostSyncService;

  const AutoEditorPage({
    super.key,
    required this.prefs,
    required this.auto,
    required this.allPaths,
    required this.allChoreoPaths,
    required this.allPathNames,
    required this.fieldImage,
    required this.onRenamed,
    required this.undoStack,
    this.shortcuts = true,
    this.telemetry,
    this.hotReload = false,
    this.ghostSyncService,
  });

  @override
  State<AutoEditorPage> createState() => _AutoEditorPageState();
}

class _AutoEditorPageState extends State<AutoEditorPage> {
  void _editPath(EditPathResult result) async {
    final path = widget.allPaths.firstWhereOrNull(
        (p) => p.name == result.pathName);
    if (path == null) return;

    widget.undoStack.clearHistory();

    await Navigator.push(
      context,
      MaterialPageRoute(
        builder: (context) => PathEditorPage(
          prefs: widget.prefs,
          path: path,
          fieldImage: widget.fieldImage,
          undoStack: widget.undoStack,
          onRenamed: (newName) {
            String oldName = path.name;
            path.renamePath(newName);
            widget.auto.updatePathName(oldName, newName);
            widget.auto.saveFile();
          },
          shortcuts: widget.shortcuts,
          telemetry: widget.telemetry,
          hotReload: widget.hotReload,
          simulatePath: true,
          ghostAutos: result.ghostAutos,
          ghostTimeOffset: result.ghostTimeOffset,
          onPathChanged: () {
            // Update linked waypoint positions across all paths
            if (path.waypoints.first.linkedName != null) {
              Waypoint.linked[path.waypoints.first.linkedName!] = Pose2d(
                  path.waypoints.first.anchor,
                  path.idealStartingState.rotation);
            }
            if (path.waypoints.last.linkedName != null) {
              Waypoint.linked[path.waypoints.last.linkedName!] = Pose2d(
                  path.waypoints.last.anchor, path.goalEndState.rotation);
            }

            for (PathPlannerPath p in widget.allPaths) {
              bool changed = false;

              for (int i = 0; i < p.waypoints.length; i++) {
                Waypoint w = p.waypoints[i];
                if (w.linkedName != null &&
                    Waypoint.linked.containsKey(w.linkedName!)) {
                  Pose2d link = Waypoint.linked[w.linkedName!]!;

                  if (link.translation.getDistance(w.anchor) >= 0.01) {
                    w.move(link.translation.x, link.translation.y);
                    changed = true;
                  }

                  if (i == 0 &&
                      (link.rotation - p.idealStartingState.rotation)
                              .degrees
                              .abs() >
                          0.01) {
                    p.idealStartingState.rotation = link.rotation;
                    changed = true;
                  } else if (i == p.waypoints.length - 1 &&
                      (link.rotation - p.goalEndState.rotation).degrees.abs() >
                          0.01) {
                    p.goalEndState.rotation = link.rotation;
                    changed = true;
                  }
                }
              }

              if (changed) {
                p.generateAndSavePath();

                if (widget.hotReload) {
                  widget.telemetry?.hotReloadPath(p);
                }
              }
            }
          },
        ),
      ),
    );

    // Rebuild auto editor to re-simulate with any path changes
    if (mounted) {
      widget.undoStack.clearHistory();
      setState(() {});
    }
  }

  @override
  Widget build(BuildContext context) {
    ColorScheme colorScheme = Theme.of(context).colorScheme;

    List<String> autoPathNames = widget.auto.getAllPathNames();
    List<PathPlannerPath> autoPaths = widget.auto.choreoAuto
        ? []
        : autoPathNames
            .map((name) =>
                widget.allPaths.firstWhere((path) => path.name == name))
            .toList();
    List<ChoreoPath> autoChoreoPaths = widget.auto.choreoAuto
        ? autoPathNames
            .map((name) =>
                widget.allChoreoPaths.firstWhere((path) => path.name == name))
            .toList()
        : [];

    final editorWidget = SplitAutoEditor(
      prefs: widget.prefs,
      auto: widget.auto,
      autoPaths: autoPaths,
      autoChoreoPaths: autoChoreoPaths,
      allPathNames: widget.allPathNames,
      fieldImage: widget.fieldImage,
      undoStack: widget.undoStack,
      ghostSyncService: widget.ghostSyncService,
      onAutoChanged: () {
        setState(() {
          widget.auto.saveFile();
        });

        if (widget.hotReload) {
          widget.telemetry?.hotReloadAuto(widget.auto);
        }
      },
      onEditPathPressed: (result) {
        _editPath(result);
      },
    );

    return Scaffold(
      appBar: CustomAppBar(
        titleWidget: RenamableTitle(
          title: widget.auto.name,
          textStyle: TextStyle(
            fontSize: 20,
            fontWeight: FontWeight.w500,
            color: colorScheme.onSurface,
          ),
          onRename: (value) {
            widget.onRenamed.call(value);
            setState(() {});
          },
        ),
        leading: BackButton(
          onPressed: () {
            widget.undoStack.clearHistory();
            Navigator.of(context).pop();
          },
        ),
      ),
      body: ConditionalWidget(
        condition: widget.shortcuts,
        trueChild: KeyBoardShortcuts(
          keysToPress: shortCut(BasicShortCuts.undo),
          onKeysPressed: widget.undoStack.undo,
          child: KeyBoardShortcuts(
            keysToPress: shortCut(BasicShortCuts.redo),
            onKeysPressed: widget.undoStack.redo,
            child: editorWidget,
          ),
        ),
        falseChild: editorWidget,
      ),
    );
  }
}
