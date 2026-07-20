"""rules.py – State machine for the drag tree sequence.

Key behaviors:
  - Staging Buffer: Waits for both cars to stage (or WO timeout) before starting.
  - Tree Sequence: 3 yellows → green, with per-lane burn detection.
  - Burn Detection: Unstage or false-movement during yellows → burn.
  - Grace Period (1s): When a car unstages during yellows, it has 1 second
    to re-stage before being burned. This prevents accidental burns from
    minor position adjustments.
  - Red Light Clearing: After green fires, an 8-second cooldown prevents
    immediate clearing. After the cooldown, if the burned driver passes
    over pre-stage, the red light turns off (so it doesn't stay on for
    the full run duration).
  - Pre-stage/Stage lights continue to reflect sensor state even after
    a burn, giving the driver visual reference.
"""
from domain.models import Lane
from infrastructure.logger import FileLogger
import math

def predict_tree_lights(elapsed, duration=2.6, green_hold_s=1.2):
    """Pure schedule function: tree bulbs for a given time since timer_start.

    Used by clients in server mode to render the countdown from the
    synchronized clock (server_time = local_time + offset) instead of
    waiting for each bulb's broadcast — so every pilot sees each bulb at
    the same real-world instant regardless of individual network latency.

    Mirrors the timing windows of RaceStateMachine.update():
      yellow 1: [duration-1.2, duration-0.8)
      yellow 2: [duration-0.8, duration-0.4)
      yellow 3: [duration-0.4, duration)
      green   : [duration, duration+green_hold_s)

    Returns (yellows_list[3], green_bool).
    """
    y1 = duration - 1.2
    y2 = duration - 0.8
    y3 = duration - 0.4
    yellows = [
        y1 <= elapsed < y2,
        y2 <= elapsed < y3,
        y3 <= elapsed < duration,
    ]
    green = duration <= elapsed < (duration + green_hold_s)
    return yellows, green

class TreeLights:
    def __init__(self):
        self.prestage = {Lane.LEFT: False, Lane.RIGHT: False}
        self.stage = {Lane.LEFT: False, Lane.RIGHT: False}
        self.yellows = {Lane.LEFT: [False, False, False], Lane.RIGHT: [False, False, False]} # 3 yellows
        self.green = {Lane.LEFT: False, Lane.RIGHT: False}
        self.red = {Lane.LEFT: False, Lane.RIGHT: False}

    def reset(self):
        self.prestage = {Lane.LEFT: False, Lane.RIGHT: False}
        self.stage = {Lane.LEFT: False, Lane.RIGHT: False}
        self.yellows = {Lane.LEFT: [False, False, False], Lane.RIGHT: [False, False, False]}
        self.green = {Lane.LEFT: False, Lane.RIGHT: False}
        self.red = {Lane.LEFT: False, Lane.RIGHT: False}

class RaceStateMachine:
    def __init__(self, false_move_threshold=0.25):
        self.lights = TreeLights()
        self.timer_start = 0.0
        self.timer_duration = 2.6
        self.current_time = 0.0
        
        # Detected sensor states
        self.car_prestage = {Lane.LEFT: False, Lane.RIGHT: False}
        self.car_stage = {Lane.LEFT: False, Lane.RIGHT: False}

        # Race control
        self.timer_running = False
        self.run_active = False
        self.pin_on_fire = False
        self.green_hold_until = {Lane.LEFT: 0.0, Lane.RIGHT: 0.0}
        
        # Constants
        self.GREEN_HOLD_SEC = 1.2
        
        # --- Staging Buffer (anti-desync) ---
        self.STAGING_BUFFER_S = 3.0   # Settle time after both staged (increased from 1.5)
        self.WO_TIMEOUT_S = 10.0      # Wait for opponent before WO
        self.NET_JITTER_TOLERANCE_S = 0.5 # MP sync tolerance
        
        # New staging logic trackers
        self.both_staged_since = None
        self.waiting_for_opponent_since = None
        self.lane_last_ready_time = {Lane.LEFT: 0.0, Lane.RIGHT: 0.0}
        
        # --- Start Pending (1.5s Alignment Grace) ---
        self.START_PENDING_S = 1.5
        self.start_pending = False
        self.start_pending_time = 0.0
        
        # --- Per-lane multiplayer state ---
        self.lane_active = {Lane.LEFT: False, Lane.RIGHT: False}
        self.lane_wo = {Lane.LEFT: False, Lane.RIGHT: False}
        self.lane_burned = {Lane.LEFT: False, Lane.RIGHT: False}
        self.lane_burn_time = {Lane.LEFT: None, Lane.RIGHT: None}
        
        # --- Burn detection state ---
        self.FALSEMOVE_DIST_M = false_move_threshold  # Tolerance for position drift while staged
        self.lane_prev_staged = {Lane.LEFT: False, Lane.RIGHT: False}
        self.lane_stage_pos0 = {Lane.LEFT: None, Lane.RIGHT: None}
        
        # --- Stage Grace Period (1s) ---
        # When a car unstages during yellows, we don't burn immediately.
        # Instead, we record when it first unstaged and give 1 second
        # for the driver to re-align before burning.
        self.UNSTAGE_GRACE_S = 0.1
        self.lane_unstage_time = {Lane.LEFT: None, Lane.RIGHT: None}
        
        # --- Red Light Clearing ---
        # cleared.
        self.RED_CLEAR_COOLDOWN_S = 8.0
        self.lane_had_full_stage = {Lane.LEFT: False, Lane.RIGHT: False}

    def update_sensor_state(self, lane, is_prestage, is_stage):
        self.car_prestage[lane] = is_prestage
        self.car_stage[lane] = is_stage
        if is_prestage and is_stage:
            self.lane_had_full_stage[lane] = True

    def reset(self):
        """Full reset of race state — clears all timers, flags, and lights."""
        self.timer_running = False
        self.run_active = False
        self.pin_on_fire = False
        self.timer_start = 0.0
        self.current_time = 0.0
        self.green_hold_until = {Lane.LEFT: 0.0, Lane.RIGHT: 0.0}
        
        # Staging buffer
        self.both_staged_since = None
        self.waiting_for_opponent_since = None
        self.lane_last_ready_time = {Lane.LEFT: 0.0, Lane.RIGHT: 0.0}
        self.start_pending = False
        self.start_pending_time = 0.0
        
        # Lane states
        self.lane_active = {Lane.LEFT: False, Lane.RIGHT: False}
        self.lane_wo = {Lane.LEFT: False, Lane.RIGHT: False}
        self.lane_burned = {Lane.LEFT: False, Lane.RIGHT: False}
        self.lane_burn_time = {Lane.LEFT: None, Lane.RIGHT: None}
        self.lane_prev_staged = {Lane.LEFT: False, Lane.RIGHT: False}
        self.lane_stage_pos0 = {Lane.LEFT: None, Lane.RIGHT: None}
        
        # Grace period
        self.lane_unstage_time = {Lane.LEFT: None, Lane.RIGHT: None}
        
        # Red light clearing
        self.lane_had_full_stage = {Lane.LEFT: False, Lane.RIGHT: False}
        
        self.lights.reset()

    def start_sequence(self, now, left_ready, right_ready):
        """
        Starts the tree sequence with awareness of which lanes are active.
        Sets WO (walkover / red) for empty lanes.
        """
        if not left_ready and not right_ready:
            return False
            
        self.timer_running = True
        self.run_active = True
        self.pin_on_fire = True
        self.timer_start = now
        self.both_staged_since = None
        self.waiting_for_opponent_since = None
        self.lights.reset()
        self.lights.stage = {k: False for k in self.lights.stage}
        
        # Left lane
        self.lane_active[Lane.LEFT] = True
        self.lane_wo[Lane.LEFT] = not left_ready
        
        # Right lane
        self.lane_active[Lane.RIGHT] = True
        self.lane_wo[Lane.RIGHT] = not right_ready
        
        # Set red light for WO lanes immediately
        if self.lane_wo[Lane.LEFT]:
            self.lights.red[Lane.LEFT] = True
        if self.lane_wo[Lane.RIGHT]:
            self.lights.red[Lane.RIGHT] = True
        
        return True

    def set_stage_position(self, lane, pos):
        """Records the position where the car was staged at tree start."""
        self.lane_stage_pos0[lane] = pos
        self.lane_prev_staged[lane] = True

    def check_burn(self, lane, current_pos, is_staged, now):
        """
        Checks if this lane has false-started (burn).

        Two triggers:
          1. Unstage: car was on the beam but left it.
             → 0.1-second grace period: prevents jitter burns but accurately traps jump starts.
          2. False movement: car still "staged" but physically
             moved > FALSEMOVE_DIST_M from its stage position.

        Returns True if a burn is detected.
        No AC dependencies — App layer provides telemetry.
        """
        # Fast fail checks
        if not self.lane_active[lane] or self.lane_wo[lane] or self.lane_burned[lane]:
            return False
        
        # Only check during yellows (pin_on_fire = tree active, not yet green)
        if not self.pin_on_fire:
            return False
        
        # 1. Unstage detection with GRACE PERIOD (debounce)
        if self.lane_prev_staged[lane] and not is_staged:
            # Car just left the beam — start grace timer if not already started
            if self.lane_unstage_time[lane] is None:
                self.lane_unstage_time[lane] = now
            
            # Check if grace period expired
            elapsed_unstaged = now - self.lane_unstage_time[lane]
            if elapsed_unstaged >= self.UNSTAGE_GRACE_S:
                # Grace expired → BURN with the timestamp they left the line
                self._set_burn_lane(lane, self.lane_unstage_time[lane])
                self.lane_prev_staged[lane] = False
                self.lane_unstage_time[lane] = None
                return True
            else:
                # Still within grace period — don't burn yet
                return False
        
        # Car is currently staged — reset grace timer if it was running
        if is_staged and self.lane_unstage_time[lane] is not None:
            self.lane_unstage_time[lane] = None  # Re-aligned successfully
        
        self.lane_prev_staged[lane] = bool(is_staged)
        
        # 2. False movement: still "staged" but moved too far
        start = self.lane_stage_pos0[lane]
        if start is None or current_pos is None:
            return False
        
        dx = current_pos[0] - start[0]
        dy = current_pos[1] - start[1]
        dz = current_pos[2] - start[2]
        dist_sq = dx*dx + dy*dy + dz*dz
        
        if dist_sq >= self.FALSEMOVE_DIST_M * self.FALSEMOVE_DIST_M:
            FileLogger.critical("BURN DETECTED: lane={} shifted {:.3f}m (limit {:.2f}m) before green".format(
                lane, math.sqrt(dist_sq), self.FALSEMOVE_DIST_M))
            self._set_burn_lane(lane, now)
            return True
        
        return False

    def _set_burn_lane(self, lane, now):
        """Internal: marks a lane as burned and sets red light.
        
        The red light stays on but can be cleared later via try_clear_red()
        after the 8-second cooldown. Pre-stage/stage lights continue to
        reflect sensor state for driver reference.
        """
        if self.lane_burned[lane]:
            return
        self.lane_burned[lane] = True
        self.lane_burn_time[lane] = now

        # Set red light, turn off green, but ALLOW yellows to animate
        self.lights.red[lane] = True
        self.lights.green[lane] = False
        # Do not overwrite yellows so tree can drop visibly

    def get_burn_rt(self, lane):
        """
        Computes the negative RT for a burned lane.
        Returns how far before green the car moved (as negative value).
        """
        burn_time = self.lane_burn_time.get(lane)
        if burn_time is None:
            return -0.001
        
        # Green fires at timer_start + timer_duration
        green_time = self.timer_start + self.timer_duration
        adv = green_time - burn_time
        if adv < 0:
            adv = 0
        return -abs(adv)

    def try_clear_red(self, lane, is_on_prestage, now):
        """Attempts to clear the red light for a burned lane.

        Rules:
          1. Lane must be burned.
          2. At least RED_CLEAR_COOLDOWN_S (8s) must have passed since
             the green light was fired (not since burn time).
          3. The driver must currently be on the pre-stage sensor.

        When cleared, the red light turns off so the driver doesn't have
        to sit with it for the rest of the run.

        Returns True if the red was cleared.
        """
        if not self.lane_burned[lane]:
            return False

        if not self.lights.red[lane]:
            return False  # Already cleared

        # Compute time since green fired
        green_time = self.timer_start + self.timer_duration
        if now - green_time < self.RED_CLEAR_COOLDOWN_S:
            return False  # Still in cooldown

        # Rule 3 (was documented but never enforced): the driver must be
        # back on the pre-stage sensor — the red must not clear by time
        # alone with the car anywhere on the track.
        if not is_on_prestage:
            return False

        self.lights.red[lane] = False
        return True

    def update(self, now, active_cars=2):
        self.current_time = now
        
        # --- Staging Buffer Logic (replaces old auto-start) ---
        if not self.run_active:
            # 1. Update last ready times based on current sensor state.
            # A car is ONLY ready if it is fully staged (Pre-stage AND Stage).
            left_staged = self.car_stage[Lane.LEFT] and self.car_prestage[Lane.LEFT]
            right_staged = self.car_stage[Lane.RIGHT] and self.car_prestage[Lane.RIGHT]

            if left_staged:
                self.lane_last_ready_time[Lane.LEFT] = now
            if right_staged:
                self.lane_last_ready_time[Lane.RIGHT] = now

            # 2. Check if players are considered ready (including jitter tolerance)
            left_ready = (now - self.lane_last_ready_time[Lane.LEFT]) <= self.NET_JITTER_TOLERANCE_S
            right_ready = (now - self.lane_last_ready_time[Lane.RIGHT]) <= self.NET_JITTER_TOLERANCE_S
            
            any_ready = left_ready or right_ready
            
            if not any_ready:
                # Nobody staged — reset all buffers
                if self.both_staged_since is not None or self.waiting_for_opponent_since is not None:
                    FileLogger.critical("RACE_STATE: Staging buffer reset (nobody ready).")
                self.both_staged_since = None
                self.waiting_for_opponent_since = None
            else:
                # ----------------
                # A) DUAL STAGE
                # ----------------
                if left_ready and right_ready:
                    # Reset "waiting for opponent" timer as it's no longer relevant
                    self.waiting_for_opponent_since = None
                    
                    if self.both_staged_since is None:
                        self.both_staged_since = now
                        FileLogger.critical("RACE_STATE: DUAL STAGE entered. Buffer starting ({:.1f}s).".format(self.STAGING_BUFFER_S))
                    
                    # Check if steady for BUFFER duration
                    if (now - self.both_staged_since) > self.STAGING_BUFFER_S:
                        if not self.start_pending:
                            FileLogger.critical("RACE_STATE: Dual staging buffer completed. Sequence start pending.")
                            self.start_pending = True
                            self.start_pending_time = now
                
                # ----------------
                # B) SOLO STAGE (Potential WO)
                # ----------------
                else:
                    # One is ready, one is not
                    if self.both_staged_since is not None:
                        FileLogger.critical("RACE_STATE: DUAL STAGE broken! Waiting for opponent again.")
                    self.both_staged_since = None # Reset dual buffer
                    
                    if self.waiting_for_opponent_since is None:
                        self.waiting_for_opponent_since = now
                        timeout_to_use = self.WO_TIMEOUT_S if active_cars > 1 else self.STAGING_BUFFER_S
                        FileLogger.critical("RACE_STATE: SOLO STAGE entered. Waiting for opponent ({:.1f}s).".format(timeout_to_use))
                    
                    # Check timeout
                    # If only 1 player connected, use short buffer instead of 8s WO wait
                    timeout_to_use = self.WO_TIMEOUT_S if active_cars > 1 else self.STAGING_BUFFER_S
                    if (now - self.waiting_for_opponent_since) > timeout_to_use:
                        if not self.start_pending:
                            FileLogger.critical("RACE_STATE: Solo stage timeout reached. Sequence start pending.")
                            self.start_pending = True
                            self.start_pending_time = now

            if self.start_pending:
                # During pending state, keep lights live (handled below)
                if (now - self.start_pending_time) >= self.START_PENDING_S:
                    # Grace period over - start race based on tolerant state
                    l_rdy = (now - self.lane_last_ready_time[Lane.LEFT]) <= self.NET_JITTER_TOLERANCE_S
                    r_rdy = (now - self.lane_last_ready_time[Lane.RIGHT]) <= self.NET_JITTER_TOLERANCE_S
                    self.start_sequence(now, left_ready=l_rdy, right_ready=r_rdy)
                    self.start_pending = False

        # Update Pre/Stage lights
        if not self.run_active:
            for lane in Lane.all():
                self.lights.prestage[lane] = self.car_prestage[lane]
                self.lights.stage[lane] = self.car_stage[lane]
            return
        
        elapsed = now - self.timer_start if self.timer_running else 0.0

        for lane in Lane.all():
            # Turn off pre/stage lights after 1.0s during tree sequence
            if self.timer_running and elapsed >= 1.0:
                self.lights.prestage[lane] = False
                self.lights.stage[lane] = False
            else:
                self.lights.prestage[lane] = self.car_prestage[lane]
                self.lights.stage[lane] = self.car_stage[lane]

        # --- Tree Sequence Logic ---
        if self.timer_running:
            for lane in Lane.all():
                # Only the lane(s) with an actual driver run the visual tree.
                # A walkover lane (nobody staged there — solo run) never got
                # a "before" from this pilot, so it has nothing to react to:
                # its yellows stay dark, only the red (set in start_sequence)
                # marks it as WO. If a second driver later stages and a real
                # dual-lane run starts, both lanes light up normally (WO is
                # only set at start_sequence() time, per lane, per run).
                if self.lane_wo[lane]:
                    self.lights.yellows[lane] = [False, False, False]
                    self.lights.green[lane] = False
                    # Still need this lane's pin_on_fire/finish-sequence
                    # bookkeeping below to use a real elapsed check, not
                    # skip the whole loop iteration.
                    if elapsed >= 2.6:
                        self.pin_on_fire = False
                    continue

                # Yellows — mesma fonte de verdade da renderizacao
                # agendada nos clientes (predict_tree_lights); evita as
                # janelas duplicarem/divergirem entre FSM e countdown local
                yellows, _ = predict_tree_lights(elapsed, self.timer_duration,
                                                 self.GREEN_HOLD_SEC)
                self.lights.yellows[lane] = list(yellows)

                # Green
                if elapsed >= 2.6:
                    self.pin_on_fire = False  # GREEN just fired
                    if self.green_hold_until[lane] == 0.0:
                        self.green_hold_until[lane] = now + self.GREEN_HOLD_SEC

                if self.green_hold_until[lane] > 0.0 and now < self.green_hold_until[lane]:
                    # Only show green if they haven't burned/WO
                    if not self.lane_burned[lane] and not self.lane_wo[lane]:
                        self.lights.green[lane] = True
                else:
                    self.lights.green[lane] = False
            
            # Finish sequence
            max_hold = max(self.green_hold_until[Lane.LEFT], self.green_hold_until[Lane.RIGHT])
            if elapsed >= 2.6 and now >= max_hold:
                self.timer_running = False
                # Reset lights for ALL lanes at the end of green hold duration
                for lane in Lane.all():
                    self.lights.yellows[lane] = [False, False, False]
                    self.lights.green[lane] = False

    # Timing Logic
    def get_race_start_time(self):
        return self.timer_start + self.timer_duration

    def get_time_to_green(self, now):
        if not self.timer_running:
            return 0.0
        return (self.timer_start + self.timer_duration) - now
