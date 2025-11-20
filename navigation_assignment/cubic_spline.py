import math
import numpy as np
import bisect

class CubicSpline1D:
    def __init__(self, x, y):
        h = np.diff(x)
        if np.any(h < 0):
            raise ValueError("x coordinates must be sorted")

        self.a = np.array(y)
        self.x = np.array(x)
        n = len(x)
        
        # Matrix solution for Spline coefficients
        A = np.zeros((n, n))
        A[0, 0] = 1.0
        A[n-1, n-1] = 1.0
        
        for i in range(1, n-1):
            A[i, i-1] = h[i-1]
            A[i, i] = 2.0 * (h[i-1] + h[i])
            A[i, i+1] = h[i]
            
        B = np.zeros(n)
        for i in range(1, n-1):
            B[i] = 3.0 * (self.a[i+1] - self.a[i]) / h[i] - 3.0 * (self.a[i] - self.a[i-1]) / h[i-1]
            
        self.c = np.linalg.solve(A, B)
        self.b = np.zeros(n-1)
        self.d = np.zeros(n-1)
        
        for i in range(n-1):
            self.d[i] = (self.c[i+1] - self.c[i]) / (3.0 * h[i])
            self.b[i] = (self.a[i+1] - self.a[i]) / h[i] - h[i] * (self.c[i+1] + 2.0 * self.c[i]) / 3.0

    def calc(self, t):
        if t < self.x[0] or t > self.x[-1]:
            return None
        i = bisect.bisect(self.x, t) - 1
        i = max(0, min(len(self.x) - 2, i))
        dx = t - self.x[i]
        return self.a[i] + self.b[i] * dx + self.c[i] * dx**2 + self.d[i] * dx**3

    def calc_d(self, t):
        if t < self.x[0] or t > self.x[-1]:
            return None
        i = bisect.bisect(self.x, t) - 1
        i = max(0, min(len(self.x) - 2, i))
        dx = t - self.x[i]
        return self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx**2

class CubicSpline2D:
    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = CubicSpline1D(self.s, x)
        self.sy = CubicSpline1D(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(ds))
        return s

    def calc_position(self, s):
        x = self.sx.calc(s)
        y = self.sy.calc(s)
        return x, y

    def calc_yaw(self, s):
        dx = self.sx.calc_d(s)
        dy = self.sy.calc_d(s)
        return math.atan2(dy, dx)