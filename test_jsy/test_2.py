import numpy as np
import math

# theta, alpha, a(r), d
test_dh_params =[[0., math.pi / 2, 0, 0.1],
                 [0., 0., -0.1, 0.],
                 [0., 0., -0.1, 0.],
                 [0., math.pi / 2, 0., 0.05],
                 [0., -math.pi / 2, 0., 0.07],
                 [0., 0., 0., 0.04]]

class Link:
    def __init__(self, dh_params):
        self.dh_params_ = dh_params

    def transformation_matrix(self):
        theta = self.dh_params_[0]
        alpha = self.dh_params_[1]
        a = self.dh_params_[2]
        d = self.dh_params_[3]
        print(d)

        st = math.sin(theta)
        ct = math.cos(theta)
        sa = math.sin(alpha)
        ca = math.cos(alpha)
        trans = np.array([[ct, -st * ca, st * sa, a * ct],
                          [st, ct * ca, -ct * sa, a * st],
                          [0, sa, ca, d],
                          [0, 0, 0, 1]])
        return trans

a=Link(test_dh_params)

a.transformation_matrix()