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
        self.lif1 = nm.LIFRecurrentCell(input_size=hidden_size, hidden_size=hidden_size, dt=1.0)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.lif2 = nm.LIFRecurrentCell(input_size=hidden_size, hidden_size=hidden_size, dt=1.0)
        self.fc3 = nn.Linear(hidden_size, action_size)
        self.apply(super().init_weights)

    def forward(self, states, s1=None, s2=None, visualize=False):
        x1 = torch.relu(self.fc1(states))
        x1, s1 = self.lif1(x1, s1)
        x2 = torch.relu(self.fc2(x1))
        x2, s2 = self.lif2(x2, s2)
        action = self.fc3(x2)
        if visualize and self.visual:
            action = torch.from_numpy(np.asarray(POSSIBLE_ACTIONS[action.argmax().tolist()], np.float32))
            self.visual.update_layers(states, action, [x1, x2], [self.lif1.synapse, self.lif2.synapse])
        return action, s1, s2

class SNN(OffPolicyAgent):
    def __init__(self, device, sim_speed):
        super().__init__(device, sim_speed)
        self.action_size = DQN_ACTION_SIZE
        self.possible_actions = POSSIBLE_ACTIONS
        self.target_update_frequency = TARGET_UPDATE_FREQUENCY
        self.epsilon = 1.0
        self.epsilon_min = 0.01
        self.epsilon_decay = 0.995
        self.discount_factor = 0.99

        self.actor = self.create_network(Actor, 'actor')
        self.actor_target = self.create_network(Actor, 'target_actor')
        self.actor_optimizer = self.create_optimizer(self.actor)

        self.hard_update(self.actor_target, self.actor)

    def get_action(self, state, is_training, step=0, visualize=False):
        if is_training and np.random.random() < self.epsilon:
            self.epsilon = max(self.epsilon_min, self.epsilon_decay * self.epsilon)
            return self.get_action_random()
        state = torch.from_numpy(np.asarray(state, np.float32)).to(self.device)
        Q_values, _, _ = self.actor(state, visualize=visualize)
        Q_values = Q_values.detach().cpu()

        if is_training:
            noise = torch.randn_like(Q_values) * 0.1  # Ajustez le facteur de bruit
            Q_values += noise

        action = Q_values.argmax().tolist()
        return action

    def get_action_random(self):
        return np.random.randint(0, self.action_size)

    def train(self, state, action, reward, state_next, done):
        action = torch.unsqueeze(action, 1)

        Q_next, _, _ = self.actor_target(state_next)
        Q_next = Q_next.amax(1, keepdim=True)
        Q_target = reward + (self.discount_factor * Q_next * (1 - done))
        Q, _, _ = self.actor(state)
        Q = Q.gather(1, action.long())
        loss = F.mse_loss(Q, Q_target)

        self.actor_optimizer.zero_grad()
        loss.backward()
        nn.utils.clip_grad_norm_(self.actor.parameters(), max_norm=1.0, norm_type=1)
        self.actor_optimizer.step()

        if self.iteration % self.target_update_frequency == 0:
            self.hard_update(self.actor_target, self.actor)
        return 0, loss.mean().detach().cpu()

    def step(self, current_position, new_position, goal_position):
        # Calculer la récompense
        reward = reward_function(current_position, new_position, goal_position)
        done = new_position == goal_position
        # Code pour obtenir l'état suivant, exécuter l'entraînement, etc.
        return reward, done

def reward_function(current_position, new_position, goal_position):
    if new_position == goal_position:
        return 10  # Grande récompense pour atteindre l'objectif
    if new_position == current_position:
        return -1  # Pénalité pour rester au même endroit
    return -0.1  # Petite pénalité pour chaque mouvement pour encourager à se déplacer
