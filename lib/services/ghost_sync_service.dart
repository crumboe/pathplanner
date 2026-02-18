import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:math';

import 'package:flutter/foundation.dart';
import 'package:pathplanner/auto/ghost_auto.dart';
import 'package:pathplanner/services/log.dart';

/// Connection state for the ghost sync service.
enum GhostSyncState {
  disabled,
  searching,
  connected,
}

/// Information about a discovered peer.
class PeerInfo {
  final String id;
  final String name;
  final InternetAddress address;
  final int wsPort;
  DateTime lastSeen;

  PeerInfo({
    required this.id,
    required this.name,
    required this.address,
    required this.wsPort,
    DateTime? lastSeen,
  }) : lastSeen = lastSeen ?? DateTime.now();
}

/// Service that enables real-time ghost auto synchronization between
/// multiple PathPlanner instances over a LAN connection.
///
/// Architecture:
/// - UDP broadcast beacons for discovery (port 5810)
/// - WebSocket for data exchange (port 5811+)
/// - Deterministic connection: lower UUID connects to higher UUID
/// - Supports multiple simultaneous peers
/// - Backpressure: skip send if one is already in-flight
class GhostSyncService extends ChangeNotifier {
  static const int _udpPort = 5810;
  static const int _defaultWsPort = 5811;
  static const Duration _beaconInterval = Duration(seconds: 2);
  static const Duration _peerTimeout = Duration(seconds: 6);
  static const Duration _pingInterval = Duration(seconds: 5);
  static const Duration _reconnectDelay = Duration(seconds: 3);
  static const Duration _ghostFadeDelay = Duration(seconds: 3);

  final String _id = _generateId();
  String _displayName;

  // State
  GhostSyncState _state = GhostSyncState.disabled;
  GhostSyncState get state => _state;
  String get displayName => _displayName;
  String get id => _id;

  // Network
  RawDatagramSocket? _udpSocket;
  HttpServer? _wsServer;
  int _actualWsPort = _defaultWsPort;
  int get wsPort => _actualWsPort;
  Timer? _beaconTimer;
  Timer? _peerCleanupTimer;
  Timer? _pingTimer;
  Timer? _reconnectTimer;

  // Multi-peer tracking
  final Map<String, PeerInfo> _discoveredPeers = {};
  final Map<String, WebSocket> _peerSockets = {};
  final Map<String, PeerInfo> _connectedPeers = {};
  final Map<String, GhostAuto> _peerGhostMap = {};
  final Map<String, Timer> _ghostFadeTimers = {};
  final Set<String> _connectingTo = {};

  /// All currently connected peers.
  Map<String, PeerInfo> get connectedPeers =>
      Map.unmodifiable(_connectedPeers);
  int get connectedPeerCount => _connectedPeers.length;

  // Keep the single-peer getter for tooltip convenience (shows first peer)
  PeerInfo? get connectedPeer =>
      _connectedPeers.isNotEmpty ? _connectedPeers.values.first : null;

  bool _isSending = false;
  bool _pendingSend = false;
  Timer? _rebuildDebounce;

  // Ghost data
  GhostAuto? _myGhost;
  final ValueNotifier<List<GhostAuto>> peerGhosts =
      ValueNotifier<List<GhostAuto>>([]);

  // Local IP for display
  String? _localIp;
  String? get localIp => _localIp;

  // Error messages for UI
  String? _lastError;
  String? get lastError => _lastError;

  // Callback for when a peer's ghost auto name changes
  VoidCallback? onPeerAutoChanged;

  // Callback for when a peer disconnects / clears ghost
  void Function(String peerName)? onPeerGhostCleared;

  GhostSyncService({required String displayName})
      : _displayName = displayName;

  /// Update the display name shown to peers.
  void setDisplayName(String name) {
    _displayName = name;
    notifyListeners();
  }

  /// Enable sync: start UDP discovery and WebSocket server.
  Future<void> enable() async {
    if (_state != GhostSyncState.disabled) return;

    _lastError = null;
    await _detectLocalIp();

    try {
      await _startUdpDiscovery();
      await _startWsServer();
    } catch (e) {
      _lastError = _formatNetworkError(e);
      Log.error('Failed to start ghost sync', e);
      await disable();
      notifyListeners();
      return;
    }

    _state = GhostSyncState.searching;
    Log.info('Ghost sync enabled (UDP: $_udpPort, WS: $_actualWsPort)');
    notifyListeners();
  }

  /// Disable sync: close all connections and stop listening.
  Future<void> disable() async {
    _beaconTimer?.cancel();
    _peerCleanupTimer?.cancel();
    _pingTimer?.cancel();
    _reconnectTimer?.cancel();

    _beaconTimer = null;
    _peerCleanupTimer = null;
    _pingTimer = null;
    _reconnectTimer = null;
    _rebuildDebounce?.cancel();
    _rebuildDebounce = null;

    for (var timer in _ghostFadeTimers.values) {
      timer.cancel();
    }
    _ghostFadeTimers.clear();

    // Snapshot keys+sockets before closing, because ws.close() can
    // trigger onDone → _handleDisconnect → _peerSockets.remove(),
    // which would cause ConcurrentModificationException.
    final socketsToClose = _peerSockets.values.toList();
    _peerSockets.clear();
    for (var ws in socketsToClose) {
      try {
        await ws.close();
      } catch (_) {}
    }
    _connectedPeers.clear();
    _connectingTo.clear();

    await _wsServer?.close(force: true);
    _wsServer = null;

    _udpSocket?.close();
    _udpSocket = null;

    _discoveredPeers.clear();
    _peerGhostMap.clear();
    _myGhost = null;
    _isSending = false;
    _pendingSend = false;

    peerGhosts.value = [];

    _state = GhostSyncState.disabled;
    Log.info('Ghost sync disabled');
    notifyListeners();
  }

  /// Publish the current ghost auto to all connected peers.
  /// Uses backpressure: if a send is in-flight, queues the latest and
  /// sends it when the current one completes.
  void publishGhost(GhostAuto? ghost) {
    _myGhost = ghost;

    if (_peerSockets.isEmpty) return;

    if (ghost == null) {
      _broadcastMessage({'type': 'ghost_clear'});
      return;
    }

    if (_isSending) {
      _pendingSend = true;
      return;
    }

    _doPublish(ghost);
  }

  /// Connect to a specific peer by IP address (manual connect fallback).
  Future<void> connectToAddress(String address, {int? port}) async {
    if (_state == GhostSyncState.disabled) return;

    int wsPort = port ?? _defaultWsPort;
    try {
      final ws = await WebSocket.connect('ws://$address:$wsPort')
          .timeout(const Duration(seconds: 5));
      _handlePeerConnection(ws, isInitiator: true, peerId: null);
    } catch (e) {
      _lastError = 'Failed to connect to $address:$wsPort';
      Log.error('Manual connect failed', e);
      notifyListeners();
    }
  }

  // ──────────────────────────────────────────────────
  // UDP Discovery
  // ──────────────────────────────────────────────────

  Future<void> _startUdpDiscovery() async {
    _udpSocket = await RawDatagramSocket.bind(
      InternetAddress.anyIPv4,
      _udpPort,
      reuseAddress: true,
      reusePort: Platform.isLinux || Platform.isMacOS,
    );
    _udpSocket!.broadcastEnabled = true;

    _udpSocket!.listen((RawSocketEvent event) {
      if (event == RawSocketEvent.read) {
        final datagram = _udpSocket!.receive();
        if (datagram != null) {
          _handleBeacon(datagram);
        }
      }
    });

    // Send beacons periodically
    _sendBeacon();
    _beaconTimer =
        Timer.periodic(_beaconInterval, (_) => _sendBeacon());

    // Clean up stale peers
    _peerCleanupTimer =
        Timer.periodic(const Duration(seconds: 2), (_) => _cleanupPeers());
  }

  void _sendBeacon() {
    if (_udpSocket == null) return;

    final beacon = jsonEncode({
      'type': 'beacon',
      'name': _displayName,
      'wsPort': _actualWsPort,
      'id': _id,
    });

    try {
      _udpSocket!.send(
        utf8.encode(beacon),
        InternetAddress('255.255.255.255'),
        _udpPort,
      );
    } catch (e) {
      Log.warning('Failed to send beacon: $e');
    }
  }

  void _handleBeacon(Datagram datagram) {
    try {
      final json = jsonDecode(utf8.decode(datagram.data));
      if (json['type'] != 'beacon') return;

      String peerId = json['id'];
      if (peerId == _id) return; // Ignore own beacons

      String peerName = json['name'] ?? 'Unknown';
      int peerWsPort = json['wsPort'] ?? _defaultWsPort;

      bool isNew = !_discoveredPeers.containsKey(peerId);
      _discoveredPeers[peerId] = PeerInfo(
        id: peerId,
        name: peerName,
        address: datagram.address,
        wsPort: peerWsPort,
      );

      if (isNew) {
        Log.info('Discovered peer: $peerName at ${datagram.address.address}:$peerWsPort');
        notifyListeners();
        _tryConnectToPeer(peerId);
      }
    } catch (e) {
      // Ignore malformed beacons
    }
  }

  void _cleanupPeers() {
    final now = DateTime.now();
    final staleIds = <String>[];

    for (var entry in _discoveredPeers.entries) {
      if (now.difference(entry.value.lastSeen) > _peerTimeout) {
        staleIds.add(entry.key);
      }
    }

    for (String id in staleIds) {
      Log.info('Peer timed out: ${_discoveredPeers[id]?.name}');
      _discoveredPeers.remove(id);
    }

    if (staleIds.isNotEmpty) {
      notifyListeners();
    }
  }

  /// Deterministic connection: lower UUID initiates.
  void _tryConnectToPeer(String peerId) {
    // Already connected or connecting to this peer
    if (_peerSockets.containsKey(peerId)) return;
    if (_connectingTo.contains(peerId)) return;
    if (_id.compareTo(peerId) >= 0) return; // Higher UUID waits

    final peer = _discoveredPeers[peerId];
    if (peer == null) return;

    _connectToPeer(peer);
  }

  Future<void> _connectToPeer(PeerInfo peer) async {
    _connectingTo.add(peer.id);
    try {
      final ws = await WebSocket.connect(
              'ws://${peer.address.address}:${peer.wsPort}')
          .timeout(const Duration(seconds: 5));
      _connectingTo.remove(peer.id);
      _handlePeerConnection(ws, isInitiator: true, peerId: peer.id);
    } catch (e) {
      _connectingTo.remove(peer.id);
      Log.warning('Failed to connect to peer ${peer.name}: $e');
      // Will retry on next beacon
    }
  }

  // ──────────────────────────────────────────────────
  // WebSocket Server
  // ──────────────────────────────────────────────────

  Future<void> _startWsServer() async {
    // Try preferred port, then auto-increment on conflict
    for (int port = _defaultWsPort; port < _defaultWsPort + 10; port++) {
      try {
        _wsServer = await HttpServer.bind(InternetAddress.anyIPv4, port);
        _actualWsPort = port;
        break;
      } on SocketException {
        if (port == _defaultWsPort + 9) {
          rethrow; // All ports exhausted
        }
        Log.warning('Port $port in use, trying ${port + 1}');
      }
    }

    _wsServer!.listen((HttpRequest request) async {
      try {
        final ws = await WebSocketTransformer.upgrade(request);
        _handlePeerConnection(ws, isInitiator: false, peerId: null);
      } catch (e) {
        Log.error('WebSocket upgrade failed', e);
      }
    });
  }

  // ──────────────────────────────────────────────────
  // Peer Connection
  // ──────────────────────────────────────────────────

  void _handlePeerConnection(WebSocket ws,
      {required bool isInitiator, required String? peerId}) {
    // Use a temporary key until we get their identity message.
    // Wrap in a list so the closure can see updates to it.
    final keyRef = [peerId ?? 'pending_${DateTime.now().microsecondsSinceEpoch}'];

    // If we already have a connection to this peer, close the duplicate
    if (peerId != null && _peerSockets.containsKey(peerId)) {
      ws.close();
      return;
    }

    _peerSockets[keyRef[0]] = ws;
    _ghostFadeTimers[keyRef[0]]?.cancel();
    _ghostFadeTimers.remove(keyRef[0]);

    // Send identity
    _sendMessageTo(ws, {
      'type': 'identity',
      'name': _displayName,
      'id': _id,
    });

    // Send current ghost if we have one
    if (_myGhost != null) {
      _sendMessageTo(ws, {
        'type': 'ghost_update',
        'ghost': _myGhost!.toJson(),
      });
    }

    // Start ping keepalive if not yet running
    _pingTimer?.cancel();
    _pingTimer = Timer.periodic(_pingInterval, (_) {
      // Use pre-encoded string to avoid jsonEncode overhead
      const pingMsg = '{"type":"ping"}';
      for (var ws in _peerSockets.values.toList()) {
        try {
          ws.add(pingMsg);
        } catch (_) {}
      }
    });

    ws.listen(
      (data) {
        _handleMessage(keyRef, ws, data);
      },
      onDone: () {
        _handleDisconnect(keyRef[0], 'Connection closed');
      },
      onError: (error) {
        _handleDisconnect(keyRef[0], 'Connection error: $error');
      },
    );

    Log.info(
        'Peer connected (initiator: $isInitiator, key: ${peerId ?? "pending"})');
  }

  void _handleMessage(List<String> keyRef, WebSocket ws, dynamic data) {
    try {
      final json = jsonDecode(data as String);
      String type = json['type'];

      switch (type) {
        case 'identity':
          String peerName = json['name'] ?? 'Unknown';
          String peerId = json['id'] ?? '';
          String oldKey = keyRef[0];

          // Reject self-connections (can happen on localhost)
          if (peerId == _id) {
            ws.close();
            return;
          }

          // Re-key from temporary to real peer ID
          if (oldKey != peerId && peerId.isNotEmpty) {
            _peerSockets.remove(oldKey);
            _peerGhostMap.remove(oldKey);
            _ghostFadeTimers[oldKey]?.cancel();
            _ghostFadeTimers.remove(oldKey);

            // Check for duplicate after re-keying
            if (_peerSockets.containsKey(peerId)) {
              ws.close();
              return;
            }
            _peerSockets[peerId] = ws;
            keyRef[0] = peerId; // Update the mutable ref so future messages use real ID
          }

          _connectedPeers[peerId] = PeerInfo(
            id: peerId,
            name: peerName,
            address: InternetAddress.anyIPv4,
            wsPort: 0,
          );

          _updateState();
          _lastError = null;
          Log.info('Peer identified: $peerName (id: $peerId)');
          notifyListeners();
          break;

        case 'ghost_update':
          if (json['ghost'] != null) {
            String realId = keyRef[0];
            String? previousName = _peerGhostMap[realId]?.name;

            GhostAuto ghost = GhostAuto.fromJson(json['ghost']);
            String peerName =
                _connectedPeers[realId]?.name ?? ghost.teamName;
            ghost = GhostAuto(
              name: ghost.name,
              teamName: peerName,
              isNetworkGhost: true,
              trajectory: ghost.trajectory,
              bumperSize: ghost.bumperSize,
              bumperOffset: ghost.bumperOffset,
              moduleLocations: ghost.moduleLocations,
              holonomic: ghost.holonomic,
            );

            _peerGhostMap[realId] = ghost;
            _ghostFadeTimers[realId]?.cancel();
            _ghostFadeTimers.remove(realId);
            _rebuildPeerGhostsList();

            if (previousName != null && previousName != ghost.name) {
              onPeerAutoChanged?.call();
            }
          }
          break;

        case 'ghost_clear':
          _startGhostFade(keyRef[0]);
          break;

        case 'ping':
          try {
            ws.add('{"type":"pong"}');
          } catch (_) {}
          break;

        case 'pong':
          break;
      }
    } catch (e) {
      Log.warning('Failed to parse sync message: $e');
    }
  }

  void _handleDisconnect(String peerKey, String reason) {
    Log.info('Peer disconnected ($peerKey): $reason');

    _peerSockets.remove(peerKey);
    PeerInfo? peer = _connectedPeers.remove(peerKey);

    // Start ghost fade for this peer
    _startGhostFade(peerKey);

    if (peer != null) {
      onPeerGhostCleared?.call(peer.name);
    }

    _updateState();

    // Stop ping timer if no peers left
    if (_peerSockets.isEmpty) {
      _pingTimer?.cancel();
      _pingTimer = null;
    }

    if (_state != GhostSyncState.disabled) {
      notifyListeners();

      // Try to reconnect to this peer after a delay
      _reconnectTimer?.cancel();
      _reconnectTimer = Timer(_reconnectDelay, () {
        for (var discovered in _discoveredPeers.values) {
          if (!_peerSockets.containsKey(discovered.id) &&
              !_connectingTo.contains(discovered.id) &&
              _id.compareTo(discovered.id) < 0) {
            _connectToPeer(discovered);
          }
        }
      });
    }
  }

  /// Gradually fade out a specific peer's ghost after disconnect/clear.
  void _startGhostFade(String peerId) {
    if (!_peerGhostMap.containsKey(peerId)) return;

    _ghostFadeTimers[peerId]?.cancel();
    _ghostFadeTimers[peerId] = Timer(_ghostFadeDelay, () {
      _peerGhostMap.remove(peerId);
      _ghostFadeTimers.remove(peerId);
      _rebuildPeerGhostsList();
    });
  }

  void _updateState() {
    if (_state == GhostSyncState.disabled) return;
    _state = _connectedPeers.isNotEmpty
        ? GhostSyncState.connected
        : GhostSyncState.searching;
  }

  void _rebuildPeerGhostsList() {
    // Debounce: coalesce rapid ghost updates into a single rebuild.
    _rebuildDebounce?.cancel();
    _rebuildDebounce = Timer(const Duration(milliseconds: 50), () {
      peerGhosts.value = _peerGhostMap.values.toList();
      // Don't call notifyListeners() here — the ValueNotifier already
      // notifies the editor via its own listener. Calling both caused
      // double setState/rebuild on every ghost update.
    });
  }

  // ──────────────────────────────────────────────────
  // Message Sending
  // ──────────────────────────────────────────────────

  void _sendMessageTo(WebSocket ws, Map<String, dynamic> message) {
    try {
      ws.add(jsonEncode(message));
    } catch (e) {
      Log.warning('Failed to send message: $e');
    }
  }

  void _broadcastMessage(Map<String, dynamic> message) {
    String encoded = jsonEncode(message);
    for (var ws in _peerSockets.values.toList()) {
      try {
        ws.add(encoded);
      } catch (e) {
        Log.warning('Failed to broadcast message: $e');
      }
    }
  }

  void _doPublish(GhostAuto ghost) {
    _isSending = true;

    final message = {
      'type': 'ghost_update',
      'ghost': ghost.toJson(),
    };

    String encoded = jsonEncode(message);
    for (var ws in _peerSockets.values.toList()) {
      try {
        ws.add(encoded);
      } catch (e) {
        Log.warning('Failed to publish ghost: $e');
      }
    }

    // WebSocket.add is synchronous for the send buffer, but we use a
    // microtask to batch rapid successive publishes.
    Future.microtask(() {
      _isSending = false;
      if (_pendingSend && _myGhost != null) {
        _pendingSend = false;
        _doPublish(_myGhost!);
      }
    });
  }

  // ──────────────────────────────────────────────────
  // Utility
  // ──────────────────────────────────────────────────

  Future<void> _detectLocalIp() async {
    try {
      for (var interface in await NetworkInterface.list(
          type: InternetAddressType.IPv4)) {
        for (var addr in interface.addresses) {
          if (!addr.isLoopback) {
            _localIp = addr.address;
            return;
          }
        }
      }
    } catch (e) {
      Log.warning('Could not detect local IP: $e');
    }
    _localIp = null;
  }

  String _formatNetworkError(Object error) {
    String msg = error.toString();
    if (msg.contains('SocketException') ||
        msg.contains('OS Error') ||
        msg.contains('Permission denied') ||
        msg.contains('Access is denied')) {
      return 'Network error — a firewall may be blocking PathPlanner. '
          'Allow PathPlanner through Windows Firewall and try again.';
    }
    if (msg.contains('Address already in use') ||
        msg.contains('EADDRINUSE')) {
      return 'Port $__actualWsPort is already in use. '
          'Close other PathPlanner instances and try again.';
    }
    return 'Network error: $msg';
  }

  // ignore the extra underscore — dart analyzer workaround for string interp
  int get __actualWsPort => _actualWsPort;

  static String _generateId() {
    final random = Random.secure();
    final bytes = List<int>.generate(16, (_) => random.nextInt(256));
    return bytes.map((b) => b.toRadixString(16).padLeft(2, '0')).join();
  }

  @override
  void dispose() {
    disable();
    peerGhosts.dispose();
    super.dispose();
  }
}
