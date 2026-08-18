import numpy as np


class KinematicVehicle:
    """
    Kinematic Vehicle Model

    State:
        x     : position x [m]
        y     : position y [m]
        theta : heading angle [rad]

    Control:
        v_L   : left wheel velocity [m/s]
        v_R   : right wheel velocity [m/s]

    Model:
        x_dot     = (v_L + v_R) / 2 * cos(theta)
        y_dot     = (v_L + v_R) / 2 * sin(theta)
        theta_dot = (v_R - v_L) / W
    """

    def __init__(
            self,
            track_width,
            dt
    ):
        """
        Parameters
        ----------
        track_width : float
            Distance between left and right wheels W [m]

        dt : float
            Sampling time [s]
        """

        self.W = track_width
        self.dt = dt


    def update(
            self,
            state,
            control
    ):
        """
        One step state update

        Parameters
        ----------
        state : numpy.ndarray
            Current state

            [
                x,
                y,
                theta
            ]

        control : numpy.ndarray
            Control input

            [
                v_L,
                v_R
            ]

        Returns
        -------
        next_state : numpy.ndarray
            Next state

            [
                x_next,
                y_next,
                theta_next
            ]
        """

        x, y, theta = state

        v_L, v_R = control


        # vehicle velocity

        v = (
            v_L
            + v_R
        ) / 2


        # continuous model

        x_dot = (
            v
            * np.cos(theta)
        )


        y_dot = (
            v
            * np.sin(theta)
        )


        theta_dot = (
            v_R
            - v_L
        ) / self.W


        # Euler integration

        x_next = (
            x
            + x_dot * self.dt
        )


        y_next = (
            y
            + y_dot * self.dt
        )


        theta_next = (
            theta
            + theta_dot * self.dt
        )


        # normalize heading

        theta_next = (
            theta_next
            + np.pi
        ) % (
            2 * np.pi
        ) - np.pi


        return np.array(
            [
                x_next,
                y_next,
                theta_next
            ]
        )