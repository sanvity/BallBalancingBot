import math

class AngleVal:
    def __init__(self, theta, phi, ch):
        # Inputs
        self.theta = math.radians(theta)  # Convert to radians
        self.phi = phi
        self.ch = ch

        # Constants (can be changed as needed)
        self.r = 0.05     # Radius or arm reach
        self.L1 = 0.04    # Link 1 length
        self.L2 = 0.06    # Link 2 length
        self.X = 0.02     # Horizontal offset, possibly fixed

        # Internal computed values
        self.ah1 = 0
        self.ah2 = 0
        self.ah3 = 0

    def calculateH(self):
        """Compute effective y-coordinate (height) of each leg based on theta and phi"""
        sin_phi = math.sin(math.radians(self.phi))  # phi is in degrees
        theta = self.theta

        self.ah1 = self.ch + self.r * math.sin(theta) * sin_phi
        self.ah2 = self.ch + self.r * math.cos(theta + math.radians(30)) * sin_phi
        self.ah3 = self.ch - self.r * math.cos(theta - math.radians(30)) * sin_phi

    def calculateAlpha(self, y):
        """Calculate the angle of a single leg based on geometry"""
        try:
            part1 = math.atan2(y, self.X)
            d = math.sqrt(self.X**2 + y**2)
            part2 = math.acos((self.X**2 + y**2 + self.L1**2 - self.L2**2) / (2 * self.L1 * d))
            alpha = part1 - part2
            return math.degrees(alpha)
        except ValueError:
            return float('nan')  # In case of domain errors

    def output(self):
        """Returns all 3 calculated angles"""
        self.calculateH()
        A1 = self.calculateAlpha(self.ah1)
        A2 = self.calculateAlpha(self.ah2)
        A3 = self.calculateAlpha(self.ah3)
        return A1, A2, A3
