
class PID:
    def __init__(self, Kp, Kd, Ki):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.error = 0
        
    def y(target, state, previous, dt):

        error = target - state
        derivative = (error - self.error)/dt
        self.integral += (self.error + error)/2 * dt
        
        Ps = self.Kp*error
        Ds = self.Kd*derivate
        Is = self.Ki*self.integral
        
        self.error = error

        return Ps + Ds + Is
