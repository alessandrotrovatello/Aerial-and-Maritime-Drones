import numpy as np
import matplotlib.pyplot as plt

from utils import *

class Drone():
    """
    This class represents a drone modeled as a double integrator

    Attributes
    ----------
    _dt : float
        The time step used for updating the drone's dynamics.
    _pose : np.ndarray
        The current position of the drone as a 2D array of shape (2,).
    _velocity : np.ndarray
        The current velocity vector of the drone as a 2D array of shape (2,).
    _acceleration : np.ndarray
        The current acceleration vector of the drone as a 2D array of shape (2,).
    _past_pos : list of np.ndarray
        A history of the drone's past positions, stored as a list of 2D arrays. 
        This is used for visualizing the drone's trajectory.
    _color : list
        A randomly generated RGB color assigned to the drone for display purposes.  
    """
    def __init__(self, init_pose: np.ndarray, dt: float) -> None:
        """
        Constructs all the necessary attributes of a Drone

        Parameters:
            init_pose (np.ndarray): 
                the initial position of the drone
            dt (float): 
                time step
        """
        self._pose = init_pose
        self._velocity = np.zeros(2)
        self._acceleration = np.zeros(2)
        self._dt = dt

        self._past_pos = []
        self._color = list(np.random.choice(range(256), size=3)/256)

    def get_position(self):
        """
        Returns the drone position

        Returns:
            _pose (np.ndarray): 
                the position of the drone 
        """
        return self._pose
    
    def get_velocity(self):
        """
        Returns the drone velocity

        Returns:
            _velocity (np.ndarray): 
                the position of the drone 
        """
        return self._velocity
    
    def u_alpha(self, qjs: np.ndarray, vjs: np.ndarray)->np.ndarray:
        """
            Returns fd the flocking term defined in equation 23.

            Parameters:
                qjs (np.ndarray): 
                    position of the neighbors robots.
                    as a 2D matrix of size (number of robots, 2). 
                vjs (np.ndarray): 
                    velocity of the neighbors robots.
                    
            Returns:
                u_alpha (np.ndarray), of size (2,)
        """
        nij = compute_nij(self._pose, qjs)
        phi_alpha_ = phi_alpha(sigma_norm(qjs-self._pose))
        gradient_based_term = np.sum(phi_alpha_ * nij, axis=0)
        aij_ = compute_aij(self._pose, qjs)
        velocity_diff = (vjs - self._velocity)
        consensus_term = np.sum(aij_ * velocity_diff, axis=0)

        u_alpha = PARAMS["c1_alpha"] * gradient_based_term + PARAMS["c2_alpha"] * consensus_term

        return u_alpha
    
    def u_beta(self, q_obs: np.ndarray, v_obs: np.ndarray)->np.ndarray:
        """
        Returns the repulsive force for obstacle avoidance presented in equation (59)

        Parameters:
            q_obs (np.ndarray): 
                position of the neighbors obstacles.
                as a 2D matrix of size (number of robots, 2). 
            v_obs (np.ndarray): 
                velocity of the neighbors obstacles.

        Returns:
            u_beta (np.ndarray), of size (2,)
    """
        gradient_based_term = 0.
        consensus_term = 0.
        # il prodotto della somma è la somma dei prodotti
        nij = compute_nij(self._pose, q_obs)
        phi_beta_ = phi_beta(sigma_norm(q_obs-self._pose))
        gradient_based_term = np.sum(phi_beta_ * nij, axis=0)
        aij_ = compute_aij_obstacles(self._pose, q_obs)
        velocity_diff = (v_obs - self._velocity)
        consensus_term = np.sum(aij_ * velocity_diff, axis=0)

        u_beta = PARAMS["c1_beta"] * gradient_based_term + PARAMS["c2_beta"] * consensus_term

        return u_beta
    
    def u_gamma(self, qr: np.ndarray, vr: np.ndarray)->np.ndarray:
        """
        Returns the navigation feedback term presented in equation 24.
        
        Parameters:
            qr (np.ndarray): 
                reference position of the group
            vr (np.ndarray): 
                reference velocity of the group.
        """
        u_gamma = -PARAMS["c1_gamma"] * sigma_1(self._pose - qr) - PARAMS["c2_gamma"] * (self._velocity - vr)
        return u_gamma

    def update_cmd(self, neighbors_pose: np.ndarray, neighbors_vel: np.ndarray, obs_pos: np.ndarray, obs_vel: np.ndarray, goal_pose: np.ndarray, goal_vel: np.ndarray):
        """
        Updates the drone's control command based on the drone's neighbors, obstacles and goal

        The computed command is then applied. The drone state is updated

        Parameters:
            neighbors_pose (np.ndarray): 
                positions of the drone's neighbors as a 2D array of shape (number of neighbors, 2).
            neighbors_vel (np.ndarray): 
                velocities of the drone's neighbors as a 2D array of shape (number of neighbors, 2).
            obs_pos (np.ndarray): 
                positions of the closest points of obstacles neighboring the drone, represented as a 2D array of shape (number of obstacles, 2). 
            obs_vel (np.ndarray): 
                velocities of the closest points of obstacles neighboring the drone, represented as a 2D array of shape (number of obstacles, 2). 
            goal_pose (np.ndarray): 
                shared goal position for all drones as a 1D array of shape (2,).
            goal_vel (np.ndarray): 
                shared goal velocity for all drones as a 1D array of shape (2,).
        """
        u_alpha = self.u_alpha(neighbors_pose, neighbors_vel)
        u_gamma = self.u_gamma(goal_pose, goal_vel)
        u_beta = self.u_beta(obs_pos, obs_vel)

        #cmd = u_alpha 
        cmd = u_alpha + u_gamma
        #cmd = u_alpha + u_gamma + u_beta

        self.update_state(cmd)
    
    def update_state(self, cmd: np.ndarray)->None:
        """
        Updates the drone state. 
        This method simulates the drone's dynamics as a double integrator
        The drone position and velocity is updated based on the provided acceleration.

        Parameters:
            cmd (np.ndarray): 
                the control command which is the acceleration of the drone. 1D vector of size (2,)
        """
        self._acceleration = cmd
        self._velocity += self._acceleration * self._dt
        self._pose += self._velocity * self._dt

        self._past_pos.append(self._pose) # for display purpose only

    def diplay_agent(self, ax: plt.Axes):
        """
        Displays the drone position and trajectory

        Parameters:
            ax (plt.Axes): 
                matplotlib graph
        """
        ax.scatter(self._pose[0], self._pose[1], 60, marker="*", color =self._color, zorder=10)
        past = np.array(self._past_pos)
        ax.plot(past[:,0], past[:,1], color=self._color, alpha=0.5)