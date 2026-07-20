class Lane:
    LEFT = "left"
    RIGHT = "right"

    # Helper to iterate if needed (not strictly an Enum anymore)
    @staticmethod
    def all():
        return [Lane.LEFT, Lane.RIGHT]
