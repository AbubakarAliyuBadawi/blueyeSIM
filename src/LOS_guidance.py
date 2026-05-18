from dataclasses import dataclass, field
import numpy as np


@dataclass
class State:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    surge_vel: float = 0.0

    def __add__(self, other: "State") -> "State":
        return State(
            x=self.x + other.x,
            y=self.y + other.y,
            z=self.z + other.z,
            pitch=self.pitch + other.pitch,
            yaw=self.yaw + other.yaw,
        )

    def __sub__(self, other: "State") -> "State":
        return State(
            x=self.x - other.x,
            y=self.y - other.y,
            z=self.z - other.z,
            pitch=self.pitch - other.pitch,
            yaw=self.yaw - other.yaw,
        )

    def as_los_array(self):
        return np.array([self.surge_vel, self.pitch, self.yaw])


@dataclass(slots=True)
class LOSParameters:
    """
    Parameters for the 3D Line-of-Sight (LOS) guidance algorithm.

    This class consist the configurable parameters used in the LOS guidance system
    for determining navigation commands in 3D space.

    Attributes:
        Δh (float): Horizontal look-ahead distance for LOS guidance [m]. 
                    Determines how far ahead the algorithm looks in the horizontal plane.
        Δv (float): Vertical look-ahead distance for LOS guidance [m]. 
                    Determines how far ahead the algorithm looks in the vertical plane.
        γh (float): Horizontal adaptive gain. Controls the responsiveness to horizontal cross-track errors.
        γv (float): Vertical adaptive gain. Controls the responsiveness to vertical cross-track errors.
        M_theta (float): Bound for parameter estimates. Constrains the maximum allowable adaptive parameter value.
        epsilon (float): Padding added to `M_theta` to compute `M_hat_theta`, providing a soft margin for parameter constraints.
        nominal_speed (float): Desired nominal forward speed [m/s].
        min_speed (float): Minimum allowable forward speed [m/s] to prevent stalling.
        dt (float): Time step for updating LOS guidance states [s].

    Properties:
        M_hat_theta (float): Computes the extended bound for parameter estimates
                             (`M_theta + epsilon`) used in adaptive parameter updates.

    """
    Δh: float = 2.0  
    Δv: float = 2.0  
    γh: float = 1.0  
    γv: float = 1.0  
    M_theta: float = 0.5  
    epsilon: float = 0.1  
    nominal_speed: float = 1.0
    min_speed: float = 0.1
    dt: float = 0.01
    
    @property
    def M_hat_theta(self) -> float:
        return self.M_theta + self.epsilon


@dataclass(slots=True)
class FilterParameters:
    """Parameters for third-order filter.

    Attributes:
        omega_diag: Natural frequencies for surge, pitch, and yaw [rad/s]
        zeta_diag: Damping ratios for surge, pitch, and yaw [-]
        
    """
    omega_diag: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0]))
    zeta_diag: np.ndarray = field(default_factory=lambda: np.array([1.0, 1.0, 1.0]))


class ThirdOrderLOSGuidance:
    """
    Simplified 3D Line-of-Sight (LOS) guidance algorithm.

    Provides control outputs for surge, pitch, and yaw to navigate towards a target,
    without adaptive parameter estimation.
    """

    def __init__(self, los_params: LOSParameters, filter_params: FilterParameters):
        self.los_params = los_params
        self.filter_params = filter_params
        self.x = np.zeros(9)  # Filter state
        self.horizontal_distance = 0.0
        self.setup_filter_matrices()

    @staticmethod
    def ssa(angle: float) -> float:
        """Smallest signed angle."""
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def setup_filter_matrices(self):
        omega = np.diag(self.filter_params.omega_diag)
        zeta = np.diag(self.filter_params.zeta_diag)
        omega_cubed = omega @ omega @ omega
        omega_squared = omega @ omega

        self.Ad = np.zeros((9, 9))
        self.Bd = np.zeros((9, 3))

        self.Ad[0:3, 3:6] = np.eye(3)
        self.Ad[3:6, 6:9] = np.eye(3)
        self.Ad[6:9, 0:3] = -omega_cubed
        self.Ad[6:9, 3:6] = -(2 * zeta + np.eye(3)) @ omega_squared
        self.Ad[6:9, 6:9] = -(2 * zeta + np.eye(3)) @ omega

        self.Bd[6:9, :] = omega_cubed

    def compute_angles(self, current: State, target: State) -> tuple[float, float]:
        """Compute azimuth and elevation angles."""
        dx = target.x - current.x
        dy = target.y - current.y
        dz = target.z - current.z

        self.horizontal_distance = np.sqrt(dx**2 + dy**2)

        # Compute azimuth angle πh
        π_h = np.arctan2(dy, dx)

        # Compute elevation angle πv
        π_v = np.arctan2(-dz, self.horizontal_distance)

        return π_h, π_v

    def compute_path_frame_errors(self, current: State, target: State) -> tuple[float, float, float]:
        """Compute errors in the path-tangential frame."""
        dx = target.x - current.x
        dy = target.y - current.y
        dz = target.z - current.z

        π_h, π_v = self.compute_angles(current, target)

        # Create rotation matrices
        R_z = np.array([
            [np.cos(π_h), -np.sin(π_h), 0],
            [np.sin(π_h), np.cos(π_h), 0],
            [0, 0, 1]
        ])

        R_y = np.array([
            [np.cos(π_v), 0, np.sin(π_v)],
            [0, 1, 0],
            [-np.sin(π_v), 0, np.cos(π_v)]
        ])

        # Compute errors in path frame
        error_n = np.array([dx, dy, dz])
        error_p = (R_y.T @ R_z.T @ error_n.reshape(3, 1)).flatten()

        return error_p[0], error_p[1], error_p[2]

    def compute_raw_los_guidance(self, current_pos: State, target_pos: State) -> State:
        """Compute raw LOS guidance commands."""
        # Compute angles and errors
        π_h, π_v = self.compute_angles(current_pos, target_pos)
        x_e, y_e, z_e = self.compute_path_frame_errors(current_pos, target_pos)

        # Compute desired heading angle ψd
        desired_yaw = π_h - np.arctan2(y_e, self.los_params.Δh)

        # Compute desired pitch angle θd
        desired_pitch = π_v + np.arctan2(z_e, self.los_params.Δv)

        # Compute desired speed based on cross-track error
        yaw_error = self.ssa(desired_yaw - current_pos.yaw)
        desired_speed = self.compute_desired_speed(yaw_error, self.horizontal_distance)

        commands = State(surge_vel=desired_speed, pitch=desired_pitch, yaw=desired_yaw)

        return commands

    def compute_desired_speed(self, yaw_error: float, distance_to_target: float) -> float:
        """Compute speed command with yaw and distance-based scaling."""
        yaw_factor = np.cos(yaw_error)
        distance_factor = min(1.0, distance_to_target / self.los_params.Δh)
        desired_speed = self.los_params.nominal_speed * yaw_factor * distance_factor
        return max(self.los_params.min_speed, desired_speed)

    def apply_reference_filter(self, commands: State) -> State:
        """Apply third-order reference filter to smooth commands."""
        x_dot = self.Ad @ self.x + self.Bd @ commands.as_los_array()
        self.x = self.x + x_dot * self.los_params.dt
        return State(surge_vel=self.x[0], pitch=self.x[1], yaw=self.x[2])

    def compute_guidance(self, current_pos: State, target_pos: State) -> State:
        """Compute filtered guidance commands."""
        raw_commands = self.compute_raw_los_guidance(current_pos, target_pos)
        filtered_commands = self.apply_reference_filter(raw_commands)
        filtered_commands.pitch = self.ssa(filtered_commands.pitch)
        filtered_commands.yaw = self.ssa(filtered_commands.yaw)
        return filtered_commands

    def reset_filter_state(self, current_state: State) -> None:
        """Reset the filter state."""
        self.x = np.zeros(9)
        current_state_array = np.array(current_state.as_los_array(), copy=True)
        self.x[0:3] = current_state_array