import torch


class DinovaDynamics:
    def __init__(self, cfg) -> None:
        self._cfg = cfg

        self._dt = cfg["mppi"]["dt"]
        self._device = cfg["device"]

    def step(self, states: torch.Tensor, actions: torch.Tensor, t=-1) -> torch.Tensor:
        
        self._dt = self._cfg["mppi"]["dt"]
        x, y, theta, vx, vy, omega = states[:, 0], states[:, 1], states[:, 2], states[:, 3], states[:, 4], states[:, 5]
        
        # Update velocity and position using bicycle model
        if actions.ndim == 3:
            # this means we calculate the cost over the entire horizon at once
            v = actions[:, :, 0]
            omega = actions[:, :, 1]

            # propagate theta based on the sampled omega sequence
            new_theta = theta.unsqueeze(1) + torch.cumsum(omega * self._dt, dim=1)
            new_theta = torch.fmod(new_theta, 2 * torch.pi)  # wrap theta between [0, 2π]

            delta_spline = torch.abs(v) * self._dt
            # new_spline = spline.unsqueeze(1) + torch.cumsum(delta_spline, dim=1)
            new_spline = 0.0

            delta_x = v * torch.cos(new_theta) * self._dt  # (400, 20)
            delta_y = v * torch.sin(new_theta) * self._dt

            new_x = x.unsqueeze(1) + torch.cumsum(delta_x, dim=1)
            new_y = y.unsqueeze(1) + torch.cumsum(delta_y, dim=1)
            new_states = torch.stack([new_x, new_y, new_theta, v, new_spline, omega], dim=-1)

        else:
            vx = actions[:, 0] 
            vy = actions[:, 1] 
            omega = actions[:, 2]
            v = actions[:, 0]

            new_x = x + (vx*torch.cos(theta) - vy*torch.sin(theta)) * self._dt
            new_y = y + (vy*torch.cos(theta) + vx*torch.sin(theta)) * self._dt
            new_theta = theta + omega * self._dt
            # new_spline = spline + torch.abs(actions[:, 0]) * self._dt
            new_states = torch.stack([new_x, new_y, new_theta, vx, vy, omega], dim=1)


        return new_states, actions
        
    
    def get_trajectory(self, state: torch.Tensor, actions: torch.Tensor):

        # Get the state of the Own Robot
        own_robot_state = state.unsqueeze(0)

        # Save current state as the first state in the list
        states_list = [own_robot_state.squeeze(0)]

        for t in range(actions.shape[0]):

            # Propagate the state of the own robot
            own_robot_state, _ = self.step(own_robot_state, actions[t].unsqueeze(0), t)
            states_list.append(own_robot_state.squeeze(0))
            # print(own_robot_state.shape)
            # print("x is: ", own_robot_state[:, 0])
            # print("y is: ", own_robot_state[:, 1])
            # print("spline is: ", own_robot_state[:, 4])
            # print("velocity is: ", actions[t])

        
        #print("states: ", states_list)
        return torch.stack(states_list)