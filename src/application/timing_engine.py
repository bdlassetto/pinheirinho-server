"""timing_engine.py — RT detection and partial times (60ft, 100m, 201m).

Zero dependency on AC — receives position/speed as parameters.
"""
import math

# Distance thresholds
DIST_60FT = 18.288
DIST_100M = 100.0
DIST_201M = 201.0

# Launch detection
RT_MOVE_DIST_M = 0.60
RT_SPEED_MIN_KMH = 2.0


class TimingEngine:
    """Handles all timing logic after green fires.

    Collaborators (injected):
      - ui_manager: UIManager (set_text, set_rt_burn_color, set_burn_indicator)
      - race_machine: RaceStateMachine (lane_burned, lane_wo, run_active)
      - lane_car_id: dict {lane -> car_id}
    """

    def __init__(self, ui_manager, race_machine, lane_car_id_ref):
        from domain.models import Lane
        self._ui = ui_manager
        self._rm = race_machine
        self._lane_car_id = lane_car_id_ref

        self.lane_timer_active = {Lane.LEFT: False, Lane.RIGHT: False}
        self.lane_timer_start = {Lane.LEFT: None, Lane.RIGHT: None}
        self.lane_green_pos0 = {Lane.LEFT: None, Lane.RIGHT: None}

        self.lane_partials = {
            Lane.LEFT:  self._new_partials(),
            Lane.RIGHT: self._new_partials(),
        }
        self.lane_stats = {
            Lane.LEFT:  self._new_stats(),
            Lane.RIGHT: self._new_stats(),
        }

        # Unstage physical sensor debounce
        self._lane_unstage_tracker = {Lane.LEFT: None, Lane.RIGHT: None}

    @staticmethod
    def _new_partials():
        # arc/prev_pos/prev_dir: distancia por COMPRIMENTO DE ARCO (segue a
        # tangente do trajeto). Ver update_partials — funciona em pista curva.
        return {"running": False, "done": False, "t0": 0.0, "p0": None, "s0": None,
                "arc": 0.0, "prev_pos": None, "prev_dir": None}

    @staticmethod
    def _new_stats():
        return {"rt": None, "rt_burn": False, "t60": None, "t100": None, "t201": None,
                "vfinal": None, "sum": None}

    def reset(self):
        from domain.models import Lane
        for ln in Lane.all():
            self.lane_timer_active[ln] = False
            self.lane_timer_start[ln] = None
            self.lane_green_pos0[ln] = None
            self.lane_partials[ln] = self._new_partials()
            self.lane_stats[ln] = self._new_stats()
            self._lane_unstage_tracker[ln] = None

    def on_green_fired(self, lane, now, car_pos, stage_origin_pos):
        """Call once per lane when green fires (pin_on_fire edge False)."""
        self.lane_timer_start[lane] = now
        self.lane_timer_active[lane] = True
        self.lane_green_pos0[lane] = car_pos
        self.lane_partials[lane] = self._new_partials()
        self.lane_partials[lane]["p0"] = stage_origin_pos

    def detect_launch(self, lane, now, pos, speed_kmh, is_staged, spline_s0,
                      on_launched, on_late_burn):
        """Detects launch event (RT) for one lane.

        Parameters:
          on_launched(lane, rt)           — callback when RT is detected
          on_late_burn(lane, now)         — callback when rt < 0 (late catch)
        """
        if not self.lane_timer_active[lane]:
            # ANTES DO VERDE. Continua rastreando a saida do feixe enquanto a
            # arvore corre. Antes isso era zerado a cada frame: quem largava
            # antes do verde tinha o instante da saida esquecido e, no verde,
            # o rastreio recomecava do zero — 0.1s depois concluia "saiu
            # agora", com launch_time == instante do verde, gerando RT 0.000
            # (largada PERFEITA) para uma largada QUEIMADA. Confirmado ao
            # vivo: piloto queimou e recebeu rt=0.0 com lane_burned=False.
            # Guardando o instante real da saida, o RT sai negativo assim que
            # o verde acende e cai no caminho de burn (on_late_burn).
            if (self._rm.run_active and self._rm.timer_running
                    and not self._rm.lane_burned.get(lane, False)
                    and not self._rm.lane_wo.get(lane, False)):
                if not is_staged:
                    if self._lane_unstage_tracker[lane] is None:
                        self._lane_unstage_tracker[lane] = now
                else:
                    self._lane_unstage_tracker[lane] = None
            else:
                self._lane_unstage_tracker[lane] = None
            return
        if self._rm.lane_burned.get(lane, False) or self._rm.lane_wo.get(lane, False):
            self._lane_unstage_tracker[lane] = None
            return

        rt_thr2 = RT_MOVE_DIST_M * RT_MOVE_DIST_M

        moved = False
        dist2 = None
        if pos and self.lane_green_pos0[lane]:
            dist2 = self._dist2(pos, self.lane_green_pos0[lane])
            moved = dist2 >= rt_thr2

        # Debounce physical sensor
        if not is_staged:
            if self._lane_unstage_tracker[lane] is None:
                self._lane_unstage_tracker[lane] = now
        else:
            self._lane_unstage_tracker[lane] = None

        unstage_time = self._lane_unstage_tracker[lane]
        debounced_unstage = (unstage_time is not None) and ((now - unstage_time) >= 0.1)

        launched = debounced_unstage or (moved and speed_kmh is not None and speed_kmh >= RT_SPEED_MIN_KMH)

        if launched:
            launch_time = unstage_time if debounced_unstage else now
            ts = self.lane_timer_start[lane]
            green_time = ts if ts is not None else launch_time
            rt = launch_time - green_time

            if rt < 0.0 and self._rm.run_active:
                # Carimba a queimada no instante REAL da saida do feixe
                # (launch_time), nao em 'now': get_burn_rt() mede o quanto
                # o piloto adiantou em relacao ao verde, entao usar 'now'
                # (ja depois do verde) zerava esse avanco.
                on_late_burn(lane, launch_time)
                return

            self.lane_stats[lane]["rt"] = rt
            self.lane_stats[lane]["rt_burn"] = False
            self.lane_timer_active[lane] = False

            self.lane_partials[lane]["running"] = True
            self.lane_partials[lane]["done"] = False
            self.lane_partials[lane]["t0"] = now
            self.lane_partials[lane]["s0"] = spline_s0
            # Zera o acumulador de arco na largada; a 1a leitura de posicao
            # em update_partials vira a origem do trajeto.
            self.lane_partials[lane]["arc"] = 0.0
            self.lane_partials[lane]["prev_pos"] = None
            self.lane_partials[lane]["prev_dir"] = None

            on_launched(lane, rt)

    def handle_burn(self, lane, now):
        """Side-effects of a burn: update stats, stop timing."""
        from domain.models import Lane
        rt_neg = self._rm.get_burn_rt(lane)
        self.lane_stats[lane]["rt"] = rt_neg
        self.lane_stats[lane]["rt_burn"] = True

        self.lane_timer_active[lane] = False
        self.lane_timer_start[lane] = None
        self.lane_green_pos0[lane] = None
        self.lane_partials[lane]["running"] = False
        self.lane_partials[lane]["done"] = True
        self._recalc_sum(lane)

    # Passo entre quadros maior que isto = teleporte (reset pra box, etc.):
    # ignora para nao inflar a distancia. A ~170 km/h e 50 Hz o passo real
    # e ~1 m; 15 m seria >2700 km/h, impossivel.
    MAX_STEP_M = 15.0

    def update_partials(self, lane, now, car_id, track_length, get_spline_fn,
                        get_pos_fn, get_speed_fn, lane_forward_fn,
                        on_split_done):
        """Updates partial times (60ft, 100m, 201m) for one lane.

        DISTANCIA POR COMPRIMENTO DE ARCO (segue a tangente do trajeto):
        acumula o deslocamento quadro-a-quadro do carro. Antes projetava
        (pos - p0) num vetor FIXO — o que so vale em reta; numa pista de
        arrancada em CURVA (ex.: Londrina) a projecao no eixo fixo cresce
        cada vez menos conforme o carro segue a curva e as parciais travavam
        no meio (visto ao vivo: 60ft em 8s, 100m/201m nunca). Somando o
        passo na direcao instantanea de cada quadro, a distancia segue
        qualquer curva sem precisar de track_length nem vetor da pista.

        Guardas: passo > MAX_STEP_M e teleporte (ignora); passo que inverte
        o sentido (produto escalar < 0 com o passo anterior) nao conta —
        assim voltar de re depois de largar nao avanca as parciais.

        Parameters (callables):
          get_pos_fn(car_id)    -> (x,y,z) or None   (usada)
          get_speed_fn(car_id)  -> float or None      (usada na vfinal)
          get_spline_fn/lane_forward_fn: mantidas por compat; nao usadas aqui.
          on_split_done(lane, key, value) — callback for each hit split
        """
        pp = self.lane_partials[lane]
        if (not pp["running"]) or pp["done"]:
            return

        pos = get_pos_fn(car_id)
        if not pos:
            return

        prev = pp.get("prev_pos")
        if prev is None:
            # 1a leitura pos-largada: fixa a origem do trajeto, distancia 0.
            pp["prev_pos"] = (pos[0], pos[2])
            dist = 0.0
        else:
            dx = pos[0] - prev[0]
            dz = pos[2] - prev[1]
            step = math.sqrt(dx * dx + dz * dz)
            if step > self.MAX_STEP_M:
                pass                     # teleporte: ignora, nao move origem
            elif step > 1e-4:
                pdir = pp.get("prev_dir")
                if pdir is None or (dx * pdir[0] + dz * pdir[1]) > 0.0:
                    pp["arc"] += step    # avanca so quando nao inverteu
                    pp["prev_dir"] = (dx / step, dz / step)
                pp["prev_pos"] = (pos[0], pos[2])
            dist = pp["arc"]

        dt = now - float(pp["t0"] or now)
        if dt < 0:
            dt = 0.0

        if self.lane_stats[lane]["t60"] is None and dist >= DIST_60FT:
            self.lane_stats[lane]["t60"] = dt
            on_split_done(lane, "t60", dt)

        if self.lane_stats[lane]["t100"] is None and dist >= DIST_100M:
            self.lane_stats[lane]["t100"] = dt
            on_split_done(lane, "t100", dt)

        if self.lane_stats[lane]["t201"] is None and dist >= DIST_201M:
            self.lane_stats[lane]["t201"] = dt
            pp["done"] = True
            pp["running"] = False

            v_kmh = get_speed_fn(car_id)
            self.lane_stats[lane]["vfinal"] = v_kmh
            self._recalc_sum(lane)
            on_split_done(lane, "t201", dt)
            on_split_done(lane, "vfinal", v_kmh)

    def _recalc_sum(self, lane):
        rt = self.lane_stats[lane]["rt"]
        t201 = self.lane_stats[lane]["t201"]
        if rt is not None and t201 is not None:
            self.lane_stats[lane]["sum"] = float(rt) + float(t201)
        else:
            self.lane_stats[lane]["sum"] = None

    @staticmethod
    def _dist2(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]
        dz = a[2] - b[2]
        return dx*dx + dy*dy + dz*dz
