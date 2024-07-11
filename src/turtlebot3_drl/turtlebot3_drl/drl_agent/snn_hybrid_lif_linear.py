import numpy as np
import torch
import torch.nn.functional as F
import torch.nn as nn
import torch.optim as optim
import norse.torch as nm
from ..common.settings import DQN_ACTION_SIZE, TARGET_UPDATE_FREQUENCY
from .off_policy_agent import OffPolicyAgent, Network

# Constantes pour les types d'actions
LINEAR = 0
ANGULAR = 1

# Définir les actions possibles
POSSIBLE_ACTIONS = [[0.3, -1.0], [0.3, -0.5], [1.0, 0.0], [0.3, 0.5], [0.3, 1.0]]

class Actor(Network):
    def __init__(self, name, state_size, action_size, hidden_size):
        super(Actor, self).__init__(name)
        self.fc1 = nn.Linear(state_size, hidden_size)
        self.lif1 = nm.LIFRecurrentCell(hidden_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, action_size)

    def forward(self, states, s1=None, s2=None, s3=None, visualize=False):
        x1 = self.fc1(states)
        #print("After x1", x1)
        s, mem = self.lif1(x1)
        #print("After s", s)
        x1 = self.fc2(s)
        #print("After last", x1)
            
        return x1

class SNN(OffPolicyAgent):
    def __init__(self, device, sim_speed):
        super().__init__(device, sim_speed)
        self.action_size = DQN_ACTION_SIZE
        self.possible_actions = POSSIBLE_ACTIONS
        self.target_update_frequency = TARGET_UPDATE_FREQUENCY

        self.actor = self.create_network(Actor, 'actor')
        self.actor_target = self.create_network(Actor, 'target_actor')
        self.actor_optimizer = self.create_optimizer(self.actor)

        self.hard_update(self.actor_target, self.actor)

    def get_action(self, state, is_training, step=0, visualize=False):
        #if is_training and np.random.random() < self.epsilon:
        #    return self.get_action_random()
        state = torch.from_numpy(np.asarray(state, np.float32)).to(self.device)
        Q_values = self.actor(state, visualize=visualize)
        Q_values = Q_values.detach().cpu()
        action = Q_values.argmax().tolist()
        return action

    def get_action_random(self):
        return np.random.randint(0, self.action_size)

    def train(self, state, action, reward, state_next, done):
        action = torch.unsqueeze(action, 1)

        Q_next = self.actor_target(state_next)
        Q_next = Q_next.amax(1, keepdim=True)
        Q_target = reward + (self.discount_factor * Q_next * (1 - done))
        Q = self.actor(state)
        Q = Q.gather(1, action.long())
        loss = F.mse_loss(Q, Q_target)

        self.actor_optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=1.0, norm_type=1)
        self.actor_optimizer.step()

        if self.iteration % self.target_update_frequency == 0:
            self.hard_update(self.actor_target, self.actor)
        return 0, loss.mean().detach().cpu()

