import torch

class DummyModel:
    def __init__(self, dt=0.1):
        self._cfg = {"mppi": {"dt": dt}}

    def step(self, states: torch.Tensor, actions: torch.Tensor, t=-1) -> torch.Tensor:
        self._dt = self._cfg["mppi"]["dt"]
        x, y, theta, v, spline, omega = states[:, 0], states[:, 1], states[:, 2], states[:, 3], states[:, 4], states[:, 5]

        if actions.ndim == 3:  # horizon case
            v = actions[:, :, 0]
            omega = actions[:, :, 1]

            # cumulative heading updates
            theta_cum = torch.cumsum(omega * self._dt, dim=1)
            new_theta = theta.unsqueeze(1) + theta_cum
            new_theta = torch.fmod(new_theta, 2 * torch.pi)

            # --- FIX: use "previous heading" for position updates (explicit Euler) ---
            theta_prev = theta.unsqueeze(1) + torch.cat(
                [torch.zeros_like(theta).unsqueeze(1), theta_cum[:, :-1]], dim=1
            )

            delta_x = v * torch.cos(theta_prev) * self._dt
            delta_y = v * torch.sin(theta_prev) * self._dt

            new_x = x.unsqueeze(1) + torch.cumsum(delta_x, dim=1)
            new_y = y.unsqueeze(1) + torch.cumsum(delta_y, dim=1)

            # spline progression
            delta_spline = torch.abs(v) * self._dt
            new_spline = spline.unsqueeze(1) + torch.cumsum(delta_spline, dim=1)

            new_states = torch.stack([new_x, new_y, new_theta, v, new_spline, omega], dim=-1)

        else:  # single step
            vx = actions[:, 0] * torch.cos(theta)
            vy = actions[:, 0] * torch.sin(theta)
            omega = actions[:, 1]
            v = actions[:, 0]

            new_x = x + vx * self._dt
            new_y = y + vy * self._dt
            new_theta = theta + omega * self._dt
            new_spline = spline + torch.abs(actions[:, 0]) * self._dt
            new_states = torch.stack([new_x, new_y, new_theta, v, new_spline, omega], dim=1)

        return new_states, actions

dt = 0.1
model = DummyModel(dt=dt)

# initial state: (x, y, theta, v, spline, omega)
states = torch.tensor([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])

# Horizon actions (N=1, T=3, 2) → (v, omega)
actions_horizon = torch.tensor([[[1.0, 0.1],
                                 [1.0, 0.1],
                                 [1.0, 0.1]]])

# Run horizon mode
new_states_horizon, _ = model.step(states, actions_horizon)

# Now simulate step-by-step with else-branch
states_step = states.clone()
stepwise_states = []
for t in range(3):
    a = actions_horizon[:, t, :]  # (N, 2)
    states_step, _ = model.step(states_step, a)  # one-step update
    stepwise_states.append(states_step)

stepwise_states = torch.stack(stepwise_states, dim=1)  # (N, T, 6)

print("Horizon mode states:")
print(new_states_horizon)

print("\nStep-by-step states:")
print(stepwise_states)

