"""race_state_adapter.py — Bridges WebSocket telemetry to RaceStateMachine + TimingEngine.

Receives per-lane telemetry dicts (pos_x, pos_z, vel, spline, is_prestage,
is_stage, timestamp) and drives the full race FSM including:
  - Staging buffer / tree sequence (via RaceStateMachine)
  - Burn detection
  - RT detection and partials (via TimingEngine)

No AC dependencies — all timing and geometry are derived from the telemetry.
"""
import os
import sys

# Allow importing domain/application modules from src/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from domain.rules import RaceStateMachine
from domain.models import Lane
from application.timing_engine import TimingEngine


class RaceStateAdapter:
    """Owns FSM + TimingEngine; accepts per-lane telemetry dicts."""

    def __init__(self, false_move_threshold=0.25):
        self.race_machine = RaceStateMachine(false_move_threshold=false_move_threshold)
        self.timing_engine = TimingEngine(
            ui_manager=None,
            race_machine=self.race_machine,
            lane_car_id_ref={Lane.LEFT: Lane.LEFT, Lane.RIGHT: Lane.RIGHT},
        )
        self.track_length = {Lane.LEFT: None, Lane.RIGHT: None}
        self._telemetry = {}       # lane -> latest telemetry dict
        self._event_now_prev = {}  # lane -> ultimo tempo-de-evento (monotonico)
        self._prev_pin_on_fire = True
        self._prev_timer_running = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def set_track_length(self, lane, length):
        """Set track length (metres) for a lane — used for spline-based distance.

        Aceita None para LIMPAR o valor: um piloto que entra sem informar
        track_length nao pode herdar o do piloto anterior naquela lane. Com
        um track_length residual, o calculo de distancia troca para o modo
        spline; se o app daquele piloto nao fornece spline (manda None/0),
        a distancia fica presa em zero e NENHUMA parcial dispara.
        """
        if length is None:
            self.track_length[lane] = None
            return
        try:
            self.track_length[lane] = float(length)
        except (TypeError, ValueError):
            self.track_length[lane] = None

    def store_telemetry(self, lane, data):
        """Store latest raw telemetry dict and update FSM sensor booleans.

        Call this each time a TELEMETRY message arrives from a client.
        The FSM sensor state is updated immediately; tick() will advance the FSM.
        """
        self._telemetry[lane] = data
        self.race_machine.update_sensor_state(
            lane,
            data.get('is_prestage', False),
            data.get('is_stage', False),
        )

    def _tel_age(self, tel, now):
        """Idade da telemetria vista pelo servidor.

        Prefere ts_srv (carimbo do cliente ja convertido para o relogio do
        SERVIDOR via offset NTP): a idade vira a latencia real do caminho,
        imune a skew de relogio entre PCs. Sem ts_srv (cliente antigo ou
        pre-sync), cai no timestamp local cru — comportamento antigo, que
        quebrava com skew > TELEMETRY_FRESH_S (piloto nunca 'staged' ou
        anti-fantasma desativado)."""
        ts = tel.get('ts_srv')
        if ts is not None:
            try:
                return now - float(ts)
            except (TypeError, ValueError):
                pass
        try:
            return now - float(tel.get('timestamp', 0.0))
        except (TypeError, ValueError):
            return self.TELEMETRY_FRESH_S + 1.0

    def _event_now(self, lane, tel, now):
        """Instante do EVENTO descrito pela telemetria vigente, no relogio
        do servidor. E o que torna RT/queima/parciais justos entre pings
        diferentes: o evento e cronometrado quando ACONTECEU no cliente
        (ts_srv), nao quando o pacote CHEGOU aqui — sem isso, ping 200ms
        somava ~0.1s deterministicos ao RT daquele piloto.

        Guardas: sanidade (|now-ts| > TS_SRV_SANITY_S descarta carimbo
        podre e usa a chegada), nunca no futuro do servidor, e monotonico
        por lane (nao regride). Sem ts_srv: comporta como antes (chegada).
        """
        t = None
        ts = tel.get('ts_srv')
        if ts is not None:
            try:
                t = float(ts)
            except (TypeError, ValueError):
                t = None
        if t is None or abs(now - t) > self.TS_SRV_SANITY_S:
            t = now
        if t > now:
            t = now
        prev = self._event_now_prev.get(lane)
        if prev is not None and t < prev:
            t = prev
        self._event_now_prev[lane] = t
        return t

    def is_lane_fully_staged(self, lane, now=None):
        """True if this lane's latest telemetry shows both pre-stage and
        stage beams lit — i.e. the driver is sitting fully staged right
        now. Used to let a lane re-arm as soon as its driver realigns,
        instead of waiting out a fixed post-race timer.

        With `now`, telemetry older than TELEMETRY_FRESH_S counts as NOT
        staged (a frozen app can't hold a lane 'aligned' forever)."""
        tel = self._telemetry.get(lane)
        if tel is None:
            return False
        if now is not None:
            if self._tel_age(tel, now) > self.TELEMETRY_FRESH_S:
                return False
        return bool(tel.get('is_prestage')) and bool(tel.get('is_stage'))

    def clear_lane(self, lane):
        """Drop a lane's telemetry and zero its beams (racer disconnected).

        Without this, a racer that vanishes while staged would leave its
        pre-stage/stage lights latched on for every remaining client.
        """
        self._telemetry.pop(lane, None)
        self._event_now_prev.pop(lane, None)
        self.race_machine.update_sensor_state(lane, False, False)
        # Zera tambem o track_length: ele pertence ao piloto que ocupava a
        # lane, nao a lane. Sem isso o proximo piloto herdava o valor e
        # podia cair no modo spline sem ter spline (parciais nunca contam).
        self.track_length[lane] = None

    TELEMETRY_FRESH_S = 1.0   # telemetria mais velha que isso = sensor apagado
    TS_SRV_SANITY_S = 2.0     # carimbo ts_srv fora disso do relogio local = descartado

    def tick(self, now, active_cars=2):
        """Run one FSM update cycle and return serialized state dict.

        Call at a fixed rate (e.g., 60 Hz) from the server event loop.
        """
        # Frescor: um app congelado (ou uma ponte que reenviou um frame
        # antigo) nao pode deixar o feixe travado aceso — telemetria sem
        # atualizacao ha mais de TELEMETRY_FRESH_S conta como sensor OFF.
        for _lane, _tel in list(self._telemetry.items()):
            if self._tel_age(_tel, now) > self.TELEMETRY_FRESH_S:
                if self.race_machine.car_prestage.get(_lane) or self.race_machine.car_stage.get(_lane):
                    self.race_machine.update_sensor_state(_lane, False, False)

        # 1. Timer-start rising-edge detection (reads state from PREVIOUS update())
        timer_running = self.race_machine.timer_running
        if timer_running and not self._prev_timer_running:
            self._on_timer_start(now)
        self._prev_timer_running = timer_running

        # 2. Advance FSM
        self.race_machine.update(now, active_cars=active_cars)

        # 3. Burn detection (only during yellows — pin_on_fire=True)
        if self.race_machine.run_active and self.race_machine.pin_on_fire:
            for lane in Lane.all():
                if not self.race_machine.lane_active[lane]:
                    continue
                if self.race_machine.lane_wo[lane] or self.race_machine.lane_burned[lane]:
                    continue
                tel = self._telemetry.get(lane)
                if tel is None:
                    continue
                pos = (tel.get('pos_x', 0.0), 0.0, tel.get('pos_z', 0.0))
                is_staged = tel.get('is_stage', False)
                # Tempo de EVENTO (ts_srv), nao de chegada: a queima e
                # carimbada no instante em que o carro saiu/moveu no
                # cliente — o avanco real nao e mais mascarado pelo ping.
                ev = self._event_now(lane, tel, now)
                if self.race_machine.check_burn(lane, pos, is_staged, ev):
                    self.timing_engine.handle_burn(lane, ev)

        # 4. Red-light clearing
        if self.race_machine.run_active:
            for lane in Lane.all():
                if not self.race_machine.lane_burned[lane]:
                    continue
                tel = self._telemetry.get(lane)
                if tel is None:
                    continue
                self.race_machine.try_clear_red(
                    lane, tel.get('is_prestage', False),
                    self._event_now(lane, tel, now))

        # 5. Green-fired falling-edge detection (after update())
        pin_on_fire = self.race_machine.pin_on_fire
        if self._prev_pin_on_fire and not pin_on_fire and self.race_machine.run_active:
            self._on_green_fired(now)
        self._prev_pin_on_fire = pin_on_fire

        # 6. RT detection
        if self.race_machine.run_active:
            for lane in Lane.all():
                tel = self._telemetry.get(lane)
                if tel is None:
                    continue
                pos = (tel.get('pos_x', 0.0), 0.0, tel.get('pos_z', 0.0))
                speed_kmh = tel.get('vel', 0.0)
                is_staged = tel.get('is_stage', False)
                spline_s0 = tel.get('spline', 0.0)

                def _on_launched(ln, rt, _lane=lane):
                    pass  # state already written to timing_engine.lane_stats

                def _on_late_burn(ln, t, _lane=lane):
                    self._handle_late_burn(_lane, t)

                # Tempo de EVENTO: o RT passa a medir quando o piloto
                # LARGOU no relogio sincronizado, nao quando o pacote
                # chegou — ping alto deixa de virar penalidade no RT.
                self.timing_engine.detect_launch(
                    lane, self._event_now(lane, tel, now), pos, speed_kmh,
                    is_staged, spline_s0,
                    _on_launched, _on_late_burn,
                )

        # 7. Partials (60ft / 100m / 201m)
        if self.race_machine.run_active:
            for lane in Lane.all():
                if self.race_machine.lane_burned.get(lane) or self.race_machine.lane_wo.get(lane):
                    continue
                tel = self._telemetry.get(lane)
                if tel is None:
                    continue
                tl = self.track_length.get(lane)
                spline_val = tel.get('spline', 0.0)
                pos = (tel.get('pos_x', 0.0), 0.0, tel.get('pos_z', 0.0))
                vel = tel.get('vel', 0.0)

                # Tempo de EVENTO tambem nas parciais: alem de casar com o
                # t0 da largada (evento), elimina o jitter de rede do dt.
                self.timing_engine.update_partials(
                    lane, self._event_now(lane, tel, now), lane, tl,
                    lambda cid, _s=spline_val: _s,
                    lambda cid, _p=pos: _p,
                    lambda cid, _v=vel: _v,
                    lambda ln: (0.0, 1.0),        # forward vector fallback
                    lambda ln, key, val: None,    # on_split: state in timing_engine
                )

        return self.serialize()

    def reset(self):
        """Full reset for the next race."""
        self.race_machine.reset()
        self.timing_engine.reset()
        self._event_now_prev = {}
        self._prev_pin_on_fire = True
        self._prev_timer_running = False

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _on_timer_start(self, now):
        """Tree sequence begins: reset timing stats and snapshot stage positions."""
        self.timing_engine.reset()
        for lane in Lane.all():
            if self.race_machine.lane_active[lane]:
                tel = self._telemetry.get(lane)
                if tel is not None:
                    pos = (tel.get('pos_x', 0.0), 0.0, tel.get('pos_z', 0.0))
                    self.race_machine.set_stage_position(lane, pos)

    def _on_green_fired(self, now):
        """Green light fires: arm timing for each active non-burned/wo lane.

        O relogio-zero do RT e o verde AGENDADO (timer_start + duration) —
        o mesmo instante que os clientes renderizam pela agenda — e nao o
        `now` do tick em que a borda foi detectada (ate ~8ms depois). Com
        os eventos carimbados em ts_srv, usar o tick aqui faria uma largada
        legitima logo apos o verde sair com RT negativo (queima falsa).
        """
        sched_green = self.race_machine.get_race_start_time()
        for lane in Lane.all():
            if self.race_machine.lane_burned.get(lane) or self.race_machine.lane_wo.get(lane):
                continue
            if not self.race_machine.lane_active.get(lane):
                continue
            tel = self._telemetry.get(lane)
            if tel is None:
                continue
            pos = (tel.get('pos_x', 0.0), 0.0, tel.get('pos_z', 0.0))
            stage_pos = self.race_machine.lane_stage_pos0.get(lane)
            self.timing_engine.on_green_fired(lane, sched_green, pos, stage_pos)

    def _handle_late_burn(self, lane, now):
        self.race_machine._set_burn_lane(lane, now)
        self.timing_engine.handle_burn(lane, now)

    # ------------------------------------------------------------------
    # Serialization
    # ------------------------------------------------------------------

    def serialize(self):
        """Return the full race state as a JSON-serializable dict."""
        rm = self.race_machine
        te = self.timing_engine

        lights = {
            lane: {
                'prestage': rm.lights.prestage[lane],
                'stage': rm.lights.stage[lane],
                'yellows': list(rm.lights.yellows[lane]),
                'green': rm.lights.green[lane],
                'red': rm.lights.red[lane],
            }
            for lane in Lane.all()
        }

        stats = {
            lane: dict(te.lane_stats[lane])
            for lane in Lane.all()
        }

        return {
            'run_active': rm.run_active,
            'timer_running': rm.timer_running,
            'pin_on_fire': rm.pin_on_fire,
            # Tree schedule (server clock). Clients with a synced clock render
            # the countdown locally from these — same real-world instant for
            # every pilot regardless of individual latency.
            'timer_start': rm.timer_start,
            'timer_duration': rm.timer_duration,
            'lane_active': {lane: rm.lane_active[lane] for lane in Lane.all()},
            'lane_wo': {lane: rm.lane_wo[lane] for lane in Lane.all()},
            'lane_burned': {lane: rm.lane_burned[lane] for lane in Lane.all()},
            'both_staged_since': rm.both_staged_since,
            'waiting_for_opponent_since': rm.waiting_for_opponent_since,
            'start_pending': rm.start_pending,
            'lights': lights,
            'stats': stats,
        }
