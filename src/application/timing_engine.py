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
        return {"running": False, "done": False, "t0": 0.0, "p0": None, "s0": None}

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
                on_late_burn(lane, now)
                return

            self.lane_stats[lane]["rt"] = rt
            self.lane_stats[lane]["rt_burn"] = False
            self.lane_timer_active[lane] = False

            self.lane_partials[lane]["running"] = True
            self.lane_partials[lane]["done"] = False
            self.lane_partials[lane]["t0"] = now
            self.lane_partials[lane]["s0"] = spline_s0

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

    def update_partials(self, lane, now, car_id, track_length, get_spline_fn,
                        get_pos_fn, get_speed_fn, lane_forward_fn,
                        on_split_done):
        """Updates partial times (60ft, 100m, 201m) for one lane.

        Parameters (callables):
          get_spline_fn(car_id) -> float or None
          get_pos_fn(car_id)    -> (x,y,z) or None
          get_speed_fn(car_id)  -> float or None
          lane_forward_fn(lane) -> (fx, fz)
          on_split_done(lane, key, value) — callback for each hit split
        """
        pp = self.lane_partials[lane]
        if (not pp["running"]) or pp["done"]:
            return

        dist = None
        s0 = pp.get("s0")
        if track_length and s0 is not None:
            s1 = get_spline_fn(car_id)
            if s1 is not None:
                d = float(s1) - float(s0)
                if d < 0.0:
                    d += 1.0
                if d < 0.0:
                    d = 0.0
                if d <= 0.5:
                    dist = d * track_length

        if dist is None:
            pos = get_pos_fn(car_id)
            if not pos:
                return
            p0 = pp["p0"]
            if not p0:
                return
            fx, fz = lane_forward_fn(lane)
            dx = pos[0] - p0[0]
            dz = pos[2] - p0[2]
            d_proj = dx * fx + dz * fz
            dist = d_proj if d_proj > 0.0 else 0.0

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
