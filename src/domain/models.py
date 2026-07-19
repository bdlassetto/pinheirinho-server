class Lane:
    LEFT = "left"
    RIGHT = "right"
    
    # Helper to iterate if needed (not strictly an Enum anymore)
    @staticmethod
    def all():
        return [Lane.LEFT, Lane.RIGHT]

class StageState:
    IDLE = 0
    PRESTAGE = 1
    STAGED = 2
    RACING = 3
    FINISHED = 4
    BURNED = 5  # Foul/Queimou

class CarData:
    def __init__(self, car_id, name, driver_name):
        self.car_id = car_id
        self.name = name
        self.driver_name = driver_name
        self.position = (0.0, 0.0, 0.0)
        self.speed_kmh = 0.0
        self.spline_pos = 0.0
        self.lane = None
        self.state = StageState.IDLE
        
        # Timing stats
        self.rt = None
        self.t60 = None
        self.t100 = None
        self.t201 = None
        self.v_final = None
        self.time_sum = None
        
        # Launch data for distance calcs
        self.start_pos = None
        self.start_spline = None

    def reset_stats(self):
        self.state = StageState.IDLE
        self.rt = None
        self.t60 = None
        self.t100 = None
        self.t201 = None
        self.v_final = None
        self.time_sum = None
        self.start_pos = None
        self.start_spline = None
