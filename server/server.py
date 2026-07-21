"""server.py — Pinheirinho2 WebSocket Race Server.

Standalone process (runs on the VPS). Accepts any number of connections:
  - racers: claim a lane (left/right), send telemetry, drive the FSM
  - spectators: receive every RACE_STATE broadcast (all four beams —
    pre-stage/stage left/right — light up from the racers' telemetry)

ROOMS: every client joins a room identified by the AC server it is playing
on (the app sends "ac_ip:port" in REGISTER as 'room'). Each room has its
own independent race FSM, lanes, pilots and broadcasts — pilots on AC
server 1 can NEVER light beams or affect races for pilots on AC server 2.
Clients that send no room land in the 'default' room (legacy behaviour).

Every client registers as a spectator first and may later upgrade to racer
by re-REGISTERing with a lane. Lane ownership is exclusive per room: a
claim for an occupied lane is answered with LANE_TAKEN and the client
stays a spectator.

Usage:
    pip install websockets
    python server/server.py

Protocol (client -> server):
    {"type": "REGISTER", "role": "spectator", "room": "1.2.3.4:8081"}
    {"type": "REGISTER", "role": "racer", "lane": "left", "room": "...",
     "track_length": 4123.5, "pilot": "Nome", "token": "..."}
    {"type": "REGISTER", "lane": "left"}              # legacy: implies racer
    {"type": "TELEMETRY", "lane": "left", "pos_x": 1.0, "pos_z": -357.0,
     "vel": 0.0, "spline": 0.45, "is_prestage": true, "is_stage": false,
     "timestamp": 1234567890.123}
    {"type": "SYNC", "t0": 1234.5}
    {"type": "CHAT", "text": "..."}
    {"type": "ADMIN_RESET", "admin_token": "..."}

Protocol (server -> client):
    {"type": "REGISTER_ACK", "role": "racer", "lane": "left"}
    {"type": "REGISTER_ACK", "role": "spectator"}
    {"type": "LANE_TAKEN", "lane": "left"}
    {"type": "AUTH_FAILED"}
    {"type": "SYNC_ACK", "t0": ..., "t1": ...}
    {"type": "RACE_STATE", "state": {...}}
    {"type": "CHAT", "from": "...", "lane": ..., "text": "..."}
    {"type": "ADMIN_RESET"}
    {"type": "OPPONENT_DISCONNECTED", "lane": "left"}
"""
import asyncio
import json
import logging
import os
import sys
import time

import websockets

# Allow running as `python server/server.py` from repo root
sys.path.insert(0, os.path.dirname(__file__))
from race_state_adapter import RaceStateAdapter

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s %(levelname)s %(message)s',
)
logger = logging.getLogger('RaceServer')

HOST = '0.0.0.0'
# Porta: em hosts tipo Pterodactyl a porta e alocada pelo painel e chega
# via env SERVER_PORT; PIN2_PORT permite override manual; padrao 8765.
PORT = int(os.environ.get('PIN2_PORT') or os.environ.get('SERVER_PORT') or 8765)

# Auth (env vars, set on the VPS):
#   PIN2_TOKEN       — optional. If set, REGISTER must carry the same
#                      'token' or the client stays unregistered.
#   PIN2_ADMIN_TOKEN — required for ADMIN_RESET. If NOT set, network
#                      resets are disabled entirely (fail-closed) so a
#                      random client can never reset races.
ACCESS_TOKEN = os.environ.get('PIN2_TOKEN') or None
ADMIN_TOKEN = os.environ.get('PIN2_ADMIN_TOKEN') or None

TICK_HZ = 120
TICK_INTERVAL = 1.0 / TICK_HZ
POST_RACE_RESET_DELAY = 5.0   # max seconds to show results before FSM resets
MIN_RESET_SETTLE_S = 0.5      # min time after race-end before an early
                              # realign-triggered reset is honoured — the
                              # burned car is still sitting on the beam at
                              # t=0, this avoids reading that as "returned"
RUN_WATCHDOG_S = 60.0         # max run duration — force reset if a lane never
                              # finishes (racer disconnected / crashed mid-run)
RESULTS_DIR = os.path.join(os.path.dirname(__file__), 'results')

LANES = ('left', 'right')
DEFAULT_ROOM = 'default'
MAX_ROOMS = 50   # protecao contra criacao ilimitada de salas por clientes


class Room:
    """One isolated race: own FSM, lanes, pilots and broadcast state."""

    def __init__(self, key):
        self.key = key
        self.clients = {}       # websocket -> info dict (shared with server)
        self.lanes = {}         # lane -> websocket (racers only)
        self.lane_pilots = {}   # lane -> pilot display name
        self.adapter = RaceStateAdapter()
        self.last_broadcast = None
        self.race_ended_at = None
        self.run_started_at = None
        # Early-reset-on-realign: lanes that need to re-stage before the
        # next run, and which of those we've actually seen LEAVE the beam
        # since the race ended (so a driver still sitting on the line from
        # the burn itself can't instantly flicker the tree back down).
        self.finished_lanes = set()
        self.lanes_confirmed_departed = set()


class RaceServer:
    def __init__(self):
        self.rooms = {}   # room_key -> Room

        if not os.path.exists(RESULTS_DIR):
            os.makedirs(RESULTS_DIR)

    # ------------------------------------------------------------------
    # Rooms
    # ------------------------------------------------------------------

    def _get_room(self, key):
        room = self.rooms.get(key)
        if room is None:
            room = Room(key)
            self.rooms[key] = room
            logger.info("Room created: %s", key)
        return room

    def _leave_room(self, websocket, info):
        """Remove a connection from its room, releasing its lane."""
        room_key = info.get('room')
        if room_key is None:
            return None
        room = self.rooms.get(room_key)
        if room is None:
            info['room'] = None
            return None
        room.clients.pop(websocket, None)
        lane = info.get('lane')
        freed_lane = None
        if lane is not None and room.lanes.get(lane) is websocket:
            del room.lanes[lane]
            room.adapter.clear_lane(lane)
            room.lane_pilots.pop(lane, None)
            freed_lane = lane
        info['room'] = None
        info['lane'] = None
        if not room.clients:
            del self.rooms[room_key]
            logger.info("Room removed (empty): %s", room_key)
            return None   # nobody left to notify
        return (room, freed_lane)

    # ------------------------------------------------------------------
    # Connection handling
    # ------------------------------------------------------------------

    async def handle_client(self, websocket):
        info = {'role': None, 'lane': None, 'room': None}
        peer = getattr(websocket, 'remote_address', None)
        logger.info("Client connected: %s", peer)
        try:
            async for raw in websocket:
                try:
                    msg = json.loads(raw)
                except (json.JSONDecodeError, ValueError):
                    logger.warning("Bad JSON from %s", peer)
                    continue

                mtype = msg.get('type')
                if mtype == 'REGISTER':
                    await self._handle_register(websocket, info, msg)
                elif mtype == 'TELEMETRY':
                    room = self.rooms.get(info.get('room'))
                    if room and info['role'] == 'racer' and info['lane'] is not None:
                        room.adapter.store_telemetry(info['lane'], msg)
                elif mtype == 'CHAT':
                    # Low-latency app chat: relayed to the client's ROOM only
                    room = self.rooms.get(info.get('room'))
                    text = str(msg.get('text', ''))[:300]
                    if room and text and info['role'] is not None:
                        # Remetente confiavel: nome registrado na conexao
                        # (REGISTER), nunca o campo 'from' da mensagem —
                        # espectador nao consegue se passar por outro nome
                        sender = (room.lane_pilots.get(info.get('lane'))
                                  or info.get('pilot') or info['role'])
                        await self._broadcast(room, {'type': 'CHAT',
                                                     'from': str(sender)[:64],
                                                     'lane': info.get('lane'),
                                                     'text': text})
                elif mtype == 'ADMIN_RESET':
                    # Fail-closed: only honoured with PIN2_ADMIN_TOKEN set and
                    # matching. Resets ONLY the sender's room.
                    room = self.rooms.get(info.get('room'))
                    if ADMIN_TOKEN is None:
                        logger.warning("ADMIN_RESET rejected (%s): PIN2_ADMIN_TOKEN not configured.", peer)
                    elif msg.get('admin_token') != ADMIN_TOKEN:
                        logger.warning("ADMIN_RESET rejected (%s): bad admin token.", peer)
                    elif room is not None:
                        logger.warning("ADMIN_RESET accepted (%s) room=%s — resetting FSM.", peer, room.key)
                        room.adapter.reset()
                        room.race_ended_at = None
                        room.run_started_at = None
                        room.finished_lanes = set()
                        room.lanes_confirmed_departed = set()
                        room.last_broadcast = None
                        await self._broadcast(room, {'type': 'ADMIN_RESET'})
                elif mtype == 'SYNC':
                    # Clock-sync probe: echo t0, stamp server time. Answered
                    # inline for minimal latency (accuracy of the tree
                    # schedule on the clients depends on this).
                    await self._send(websocket, {
                        'type': 'SYNC_ACK',
                        't0': msg.get('t0'),
                        't1': time.time(),
                    })

        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            logger.exception("Error in handle_client %s: %s", peer, e)
        finally:
            left = self._leave_room(websocket, info)
            if left is not None:
                room, freed_lane = left
                if freed_lane is not None:
                    logger.info("Racer disconnected: room=%s lane=%s", room.key, freed_lane)
                    await self._broadcast(room, {'type': 'OPPONENT_DISCONNECTED',
                                                 'lane': freed_lane})
            logger.info("Client disconnected: %s (role=%s)", peer, info.get('role'))

    async def _handle_register(self, websocket, info, msg):
        """Handle initial registration, room joins and racer upgrades."""
        # Optional access gate: with PIN2_TOKEN set, only clients whose
        # server.ini carries the same token get registered at all.
        if ACCESS_TOKEN is not None and msg.get('token') != ACCESS_TOKEN:
            logger.warning("REGISTER rejected: bad/missing access token.")
            await self._send(websocket, {'type': 'AUTH_FAILED'})
            return

        room_key = str(msg.get('room') or DEFAULT_ROOM)[:80]

        role = msg.get('role')
        lane = msg.get('lane')
        if role is None:
            # Legacy clients send only a lane — treat as racer
            role = 'racer' if lane in LANES else 'spectator'

        # Guarda o pilot informado na conexao (usado como remetente
        # confiavel do CHAT, inclusive para espectadores)
        pilot_field = msg.get('pilot')
        if pilot_field:
            info['pilot'] = str(pilot_field)[:64]

        # No-op re-REGISTER (keepalive de papel do cliente, a cada ~8s):
        # mesma conexao, mesma sala, mesmo papel e mesma lane — responde um
        # ACK silencioso sem log INFO nem reenvio do estado completo, para
        # nao poluir o log nem gastar banda a toa.
        if (info.get('room') == room_key and info.get('role') == role
                and (role != 'racer' or info.get('lane') == lane)):
            ack = {'type': 'REGISTER_ACK', 'role': role}
            if role == 'racer':
                ack['lane'] = lane
            await self._send(websocket, ack)
            return

        # Room cap: recusa sala NOVA acima do limite (salas existentes
        # continuam aceitando entradas normalmente)
        if room_key not in self.rooms and len(self.rooms) >= MAX_ROOMS:
            logger.warning("REGISTER rejected: room cap reached (%d), room=%s",
                           MAX_ROOMS, room_key)
            await self._send(websocket, {'type': 'AUTH_FAILED'})
            return

        # Room switch (e.g. pilot changed AC server): leave the old room
        if info.get('room') is not None and info['room'] != room_key:
            left = self._leave_room(websocket, info)
            if left is not None:
                old_room, freed_lane = left
                if freed_lane is not None:
                    await self._broadcast(old_room, {'type': 'OPPONENT_DISCONNECTED',
                                                     'lane': freed_lane})

        room = self._get_room(room_key)
        room.clients[websocket] = info
        info['room'] = room_key

        if role == 'racer':
            if lane not in LANES:
                await self._send(websocket, {'type': 'LANE_TAKEN', 'lane': lane})
                return
            owner = room.lanes.get(lane)
            if owner is not None and owner is not websocket and owner in room.clients:
                logger.info("Lane claim DENIED (taken): room=%s lane=%s", room.key, lane)
                await self._send(websocket, {'type': 'LANE_TAKEN', 'lane': lane})
                if info['role'] is None:
                    info['role'] = 'spectator'
                    await self._send(websocket, {'type': 'REGISTER_ACK', 'role': 'spectator'})
                    await self._send_current_state(room, websocket)
                return

            # Release a previously held lane if switching sides
            prev = info.get('lane')
            if prev is not None and prev != lane and room.lanes.get(prev) is websocket:
                del room.lanes[prev]
                room.adapter.clear_lane(prev)
                room.lane_pilots.pop(prev, None)

            info['role'] = 'racer'
            info['lane'] = lane
            room.lanes[lane] = websocket

            pilot = msg.get('pilot')
            if pilot:
                room.lane_pilots[lane] = str(pilot)[:64]

            # Sempre grava o track_length da lane — INCLUSIVE None. Antes so
            # gravava quando vinha um valor, entao um piloto que entrasse sem
            # track_length herdava o do piloto anterior naquela lane; com esse
            # residuo o calculo de parciais troca para o modo spline e, sem
            # spline do app, trava em distancia zero (nenhuma parcial conta).
            track_length = msg.get('track_length')
            try:
                room.adapter.set_track_length(
                    lane, None if track_length is None else float(track_length))
            except (TypeError, ValueError):
                room.adapter.set_track_length(lane, None)

            logger.info("REGISTERED racer room=%s lane=%s pilot=%s track_length=%s",
                        room.key, lane, pilot, track_length)
            await self._send(websocket, {'type': 'REGISTER_ACK', 'role': 'racer', 'lane': lane})
            await self._send_current_state(room, websocket)
            return

        # Spectator registration (also: racer downgrading to spectator)
        prev = info.get('lane')
        if prev is not None and room.lanes.get(prev) is websocket:
            del room.lanes[prev]
            room.adapter.clear_lane(prev)
            room.lane_pilots.pop(prev, None)
            await self._broadcast(room, {'type': 'OPPONENT_DISCONNECTED', 'lane': prev})
        info['role'] = 'spectator'
        info['lane'] = None
        logger.info("REGISTERED spectator room=%s (%d clients, %d racers)",
                    room.key, len(room.clients), len(room.lanes))
        await self._send(websocket, {'type': 'REGISTER_ACK', 'role': 'spectator'})
        await self._send_current_state(room, websocket)

    def _with_pilots(self, room, state):
        """Attach the per-lane pilot names to a state dict (in place)."""
        state['pilots'] = {ln: room.lane_pilots.get(ln) for ln in LANES}
        return state

    async def _send_current_state(self, room, websocket):
        """Send the room's current state so new clients never wait."""
        current_state = self._with_pilots(room, room.adapter.serialize())
        await self._send(websocket, {'type': 'RACE_STATE', 'state': current_state})

    async def _send(self, websocket, msg):
        try:
            await websocket.send(json.dumps(msg))
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Tick loop
    # ------------------------------------------------------------------

    async def tick_loop(self):
        """120 Hz FSM update loop — every room independently."""
        logger.info("Tick loop started at %d Hz", TICK_HZ)
        while True:
            t0 = time.time()
            for room in list(self.rooms.values()):
                await self._tick_room(room, t0)
            elapsed = time.time() - t0
            await asyncio.sleep(max(0.0, TICK_INTERVAL - elapsed))

    async def _tick_room(self, room, now):
        # Only racers count as active cars (spectators must not affect
        # the WO/solo staging timeout logic).
        active_cars = len(room.lanes)
        state = self._with_pilots(room, room.adapter.tick(now, active_cars=active_cars))

        # Run watchdog: without it, a racer that disconnects (or stops)
        # mid-run leaves run_active stuck forever in this room.
        if state.get('run_active'):
            if room.run_started_at is None:
                room.run_started_at = now
            elif (now - room.run_started_at) > RUN_WATCHDOG_S and room.race_ended_at is None:
                logger.warning("Run watchdog room=%s: race never finished after %.0fs — forcing reset.",
                               room.key, RUN_WATCHDOG_S)
                self._save_result(room, state)
                room.adapter.reset()
                room.run_started_at = None
                room.finished_lanes = set()
                room.lanes_confirmed_departed = set()
                room.last_broadcast = None
                return
        else:
            room.run_started_at = None

        # Race-end detection: all active, non-wo lanes have either t201 or burned
        if state.get('run_active') and not state.get('pin_on_fire') and not state.get('timer_running'):
            active_lanes = [
                ln for ln in LANES
                if state['lane_active'].get(ln) and not state['lane_wo'].get(ln)
            ]
            if active_lanes and all(
                state['stats'][ln].get('t201') is not None or state['lane_burned'].get(ln)
                for ln in active_lanes
            ):
                if room.race_ended_at is None:
                    room.race_ended_at = now
                    room.finished_lanes = set(active_lanes)
                    room.lanes_confirmed_departed = set()
                    logger.info("Race done (room=%s). Reset in up to %.0fs (sooner if driver(s) realign).",
                                room.key, POST_RACE_RESET_DELAY)
                    self._save_result(room, state)

        if room.race_ended_at is not None:
            # Track which finished lanes have actually left the beam since
            # the race ended — required before an early reset counts a
            # lane as "back and ready" (prevents the still-parked-from-the-
            # burn car from instantly re-arming the tree).
            for ln in room.finished_lanes:
                if not room.adapter.is_lane_fully_staged(ln, now):
                    room.lanes_confirmed_departed.add(ln)

            elapsed_since_end = now - room.race_ended_at
            realigned_early = (
                elapsed_since_end >= MIN_RESET_SETTLE_S
                and room.finished_lanes
                and room.finished_lanes <= room.lanes_confirmed_departed
                and all(room.adapter.is_lane_fully_staged(ln, now) for ln in room.finished_lanes)
            )
            if realigned_early or elapsed_since_end >= POST_RACE_RESET_DELAY:
                logger.info("Resetting FSM (room=%s)%s.", room.key,
                            " — driver realigned" if realigned_early else "")
                room.adapter.reset()
                room.race_ended_at = None
                room.finished_lanes = set()
                room.lanes_confirmed_departed = set()
                room.last_broadcast = None

        # Only broadcast on state change
        if state == room.last_broadcast:
            return
        self._log_transitions(room, room.last_broadcast, state)
        room.last_broadcast = state

        await self._broadcast(room, {'type': 'RACE_STATE', 'state': state})

    def _log_transitions(self, room, prev, cur):
        """Log human-readable tree events when notable state changes occur."""
        if prev is None:
            return
        prev_lights = prev.get('lights', {}).get('left', {})
        cur_lights  = cur.get('lights',  {}).get('left', {})

        # run_active rising edge
        if not prev.get('run_active') and cur.get('run_active'):
            logger.info("[%s] >>> RUN ACTIVE — tree armed", room.key)

        # beam transitions (pre-stage / stage, both lanes)
        for lane in LANES:
            p = prev.get('lights', {}).get(lane, {})
            c = cur.get('lights',  {}).get(lane, {})
            for beam in ('prestage', 'stage'):
                if not p.get(beam) and c.get(beam):
                    logger.info("[%s] >>> %s %s ON", room.key, beam.upper(), lane.upper())
                elif p.get(beam) and not c.get(beam):
                    logger.info("[%s] >>> %s %s off", room.key, beam.upper(), lane.upper())

        # yellows (track each bulb independently)
        prev_y = prev_lights.get('yellows', [False, False, False])
        cur_y  = cur_lights.get('yellows',  [False, False, False])
        for i, label in enumerate(['AMARELO 1', 'AMARELO 2', 'AMARELO 3']):
            if not prev_y[i] and cur_y[i]:
                logger.info("[%s] >>> %s", room.key, label)

        # green rising edge
        if not prev_lights.get('green') and cur_lights.get('green'):
            logger.info("[%s] >>> VERDE ← largada!", room.key)

        # RT registered for either lane
        for lane in LANES:
            prev_rt = prev.get('stats', {}).get(lane, {}).get('rt')
            cur_rt  = cur.get('stats',  {}).get(lane, {}).get('rt')
            if prev_rt is None and cur_rt is not None:
                burn = cur.get('stats', {}).get(lane, {}).get('rt_burn', False)
                tag  = ' [QUEIMA]' if burn else ''
                logger.info("[%s] >>> RT %s = %.4fs%s", room.key, lane.upper(), cur_rt, tag)

        # t201 registered for either lane
        for lane in LANES:
            prev_t = prev.get('stats', {}).get(lane, {}).get('t201')
            cur_t  = cur.get('stats',  {}).get(lane, {}).get('t201')
            if prev_t is None and cur_t is not None:
                logger.info("[%s] >>> 201m %s = %.4fs", room.key, lane.upper(), cur_t)

    def _save_result(self, room, state):
        """Save the final race state to a JSON file."""
        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            safe_room = "".join(c if c.isalnum() or c in "._-" else "_" for c in room.key)[:48]
            filename = "result_{}_{}.json".format(safe_room, timestamp)
            filepath = os.path.join(RESULTS_DIR, filename)

            result = {
                'timestamp': time.time(),
                'date': time.ctime(),
                'room': room.key,
                'state': state
            }

            with open(filepath, 'w') as f:
                json.dump(result, f, indent=2)
            logger.info("Result saved to %s", filepath)
        except Exception as e:
            logger.error("Failed to save result: %s", e)

    async def _broadcast(self, room, msg):
        if not room.clients:
            return
        raw = json.dumps(msg)
        dead = []
        for ws in list(room.clients.keys()):
            try:
                await ws.send(raw)
            except Exception:
                dead.append(ws)
        for ws in dead:
            info = room.clients.pop(ws, None)
            if info and info.get('lane') is not None and room.lanes.get(info['lane']) is ws:
                del room.lanes[info['lane']]
                room.adapter.clear_lane(info['lane'])
                room.lane_pilots.pop(info['lane'], None)
            logger.warning("Dead client removed during broadcast (room=%s)", room.key)
        # Sala que esvaziou por clientes mortos: sem isso ela ficaria
        # tickando a 120Hz para sempre (leak de CPU/memoria)
        if dead and not room.clients and self.rooms.get(room.key) is room:
            del self.rooms[room.key]
            logger.info("Room removed (empty, dead-broadcast): %s", room.key)


async def main():
    race_server = RaceServer()
    async with websockets.serve(
        race_server.handle_client, HOST, PORT,
        ping_interval=20, ping_timeout=10,
    ):
        logger.info("Pinheirinho2 Race Server on ws://%s:%d", HOST, PORT)
        tick_task = asyncio.create_task(race_server.tick_loop())
        try:
            await asyncio.Future()
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass
        finally:
            tick_task.cancel()
    logger.info("Server shut down.")


if __name__ == '__main__':
    asyncio.run(main())
