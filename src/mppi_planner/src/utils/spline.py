import torch

class TorchSpline:
    def __init__(self, x, y):
        device = "cuda:0" if torch.cuda.is_available() else "cpu"
        self.device = device
        # self.x = torch.tensor(x, dtype=torch.float32, device=device)
        # self.y = torch.tensor(y, dtype=torch.float32, device=device)
        self.x = x
        self.y = y
        self.num_x = len(x)

        h = self.x[1:] - self.x[:-1]
        self.a = self.y.clone()

        A = self._calculate_matrix_A(h)
        B = self._calculate_matrix_B(h)
        C = torch.linalg.solve(A, B)
        self.c = C.flatten()

        self.b = (self.a[1:] - self.a[:-1]) / h - h * (self.c[1:] + 2.0 * self.c[:-1]) / 3.0
        self.d = (self.c[1:] - self.c[:-1]) / (3.0 * h)

    def _calculate_matrix_A(self, h):
        A = torch.zeros((self.num_x, self.num_x), dtype=torch.float32, device=self.device)
        A[0, 0] = 1.0
        A[-1, -1] = 1.0
        for i in range(1, self.num_x - 1):
            A[i, i - 1] = h[i - 1]
            A[i, i] = 2.0 * (h[i - 1] + h[i])
            A[i, i + 1] = h[i]
        return A

    def _calculate_matrix_B(self, h):
        B = torch.zeros((self.num_x, 1), dtype=torch.float32, device=self.device)
        for i in range(1, self.num_x - 1):
            B[i] = 3.0 * ((self.a[i + 1] - self.a[i]) / h[i] - (self.a[i] - self.a[i - 1]) / h[i - 1])
        return B

    def _search_indices(self, t):
        # Find the interval indices for each t
        indices = torch.searchsorted(self.x, t.contiguous(), right=True) - 1
        indices = torch.clamp(indices, 0, self.num_x - 2)
        return indices

    def calculate_point(self, t):
        t = torch.tensor(t, device=self.device, dtype=torch.float32) if not torch.is_tensor(t) else t.to(self.device)
        i = self._search_indices(t)
        dx = t - self.x[i]
        return self.a[i] + self.b[i] * dx + self.c[i] * dx**2 + self.d[i] * dx**3

    def calculate_first_derivative(self, t):
        t = torch.tensor(t, device=self.device, dtype=torch.float32) if not torch.is_tensor(t) else t.to(self.device)
        i = self._search_indices(t)
        dx = t - self.x[i]
        return self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx**2

class TorchSpline2D:
    def __init__(self, path, device="cuda:0"):
        print("FIT A SPLINE")
        self.device = device
        self.s = self._calculate_s(path)

        x = torch.tensor(path.x, dtype=torch.float32, device=self.device)
        y = torch.tensor(path.y, dtype=torch.float32, device=self.device)

        self.sx = TorchSpline(self.s, x)
        self.sy = TorchSpline(self.s, y)
    
    def _calculate_s(self, path):
        x = torch.tensor(path.x, dtype=torch.float32, device=self.device)
        y = torch.tensor(path.y, dtype=torch.float32, device=self.device)

        dx = x[1:] - x[:-1]
        dy = y[1:] - y[:-1]

        ds = torch.sqrt(dx**2 + dy**2)
        s = torch.cat([torch.tensor([0.0], device=self.device), torch.cumsum(ds, dim=0)])
        return s

    def calculate_position(self, s):
        s = torch.tensor(s, dtype=torch.float32, device=self.device) if not torch.is_tensor(s) else s.to(self.device)
        x = self.sx.calculate_point(s)
        y = self.sy.calculate_point(s)
        return x, y

    def parameter_length(self):
        return self.s[-1]

    def deriv_normalized(self, s):
        s = torch.tensor(s, dtype=torch.float32, device=self.device) if not torch.is_tensor(s) else s.to(self.device)
        dx = self.sx.calculate_first_derivative(s)
        dy = self.sy.calculate_first_derivative(s)
        norm = torch.sqrt(dx**2 + dy**2)
        norm = torch.clamp(norm, min=1e-8)  # avoid division by 0
        return dx / norm, dy / norm 
