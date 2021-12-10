class RigidBody:
    def __init__(self):
        self.lambda_prev = None # impulses from last frame (for warm starting)
