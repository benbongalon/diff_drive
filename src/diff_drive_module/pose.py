from __future__ import division


def euler_from_quaternion(x, y, z, w):
    """Code from https://computergraphics.stackexchange.com/questions/8195/
       how-to-convert-euler-angles-to-quaternions-and-get-the-same-euler-angles-back-fr
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [roll, pitch, yaw]


class Pose:

    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        self.xVel = 0
        self.yVel = 0
        self.thetaVel = 0


    def __str__(self):
        return str({'x': self.x, 'y': self.y, 'theta': self.theta,
                    'xVel': self.xVel, 'yVel': self.yVel,
                    'thetaVel': self.thetaVel})
