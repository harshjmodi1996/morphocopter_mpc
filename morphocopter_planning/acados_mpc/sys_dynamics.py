#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#

# reference : "Towards Time-optimal Tunnel-following for Quadrotors", Jon Arrizabalaga et al.

import casadi as ca
from common import *

'''Global Symbolic variables'''
# State variables

# # x,y, z position (inertial frame, m)
x = ca.MX.sym('x')
y = ca.MX.sym('y')
z = ca.MX.sym('z')

# Transaltional velocities (inertial frame, m/s)
vx = ca.MX.sym('vx')
vy = ca.MX.sym('vy')
vz = ca.MX.sym('vz')
# v_c = ca.vertcat(vx, vy, vz)

# anglular rotations (roll, pitch, yaw - radians)
phi = ca.MX.sym('phi')
theta = ca.MX.sym('theta')
psi = ca.MX.sym('psi')

# Angular velocities w.r.t x, y, z
# (body frame, m/s)
p = ca.MX.sym('p')
q = ca.MX.sym('q')
r = ca.MX.sym('r')
# omg = ca.vertcat(wr, wp, wy)

# joint angle and joint angular velocity
J = ca.MX.sym('J')
Jdot = ca.MX.sym('Jdot')

zeta_f = ca.vertcat(x,y,z,vx,vy,vz,phi,theta,psi,p,q,r,J,Jdot)

U1 = ca.MX.sym('U1')  # thrust force of rotor 1
U2 = ca.MX.sym('U2')  # thrust force of rotor 2
U3 = ca.MX.sym('U3')  # thrust force of rotor 3
U4 = ca.MX.sym('U4')  # thrust force of rotor 4
U5 = ca.MX.sym('U5')  # torque at joint angle

u = ca.vertcat(U1, U2, U3, U4, U5) # control inputs


class SysDyn():

    def __init__(self):

        self.n_samples = 0
        self.solver = None

    def SetupOde(self):
        '''ODEs for system dynamic model'''

        # Inputs
        F1, F2, F3, F4, T_J = u[0], u[1], u[2], u[3], u[4]

        # Rotation matrices
        Z = ca.vertcat(
            ca.horzcat(ca.cos(psi), -ca.sin(psi), 0.0),
            ca.horzcat(ca.sin(psi),  ca.cos(psi), 0.0),
            ca.horzcat(0.0,            0.0,           1.0)
        )
        Y = ca.vertcat(
            ca.horzcat(ca.cos(theta), 0.0, ca.sin(theta)),
            ca.horzcat(0.0,             1.0, 0.0),
            ca.horzcat(-ca.sin(theta),0.0, ca.cos(theta))
        )
        X = ca.vertcat(
            ca.horzcat(1.0, 0.0, 0.0),
            ca.horzcat(0.0, ca.cos(phi), -ca.sin(phi)),
            ca.horzcat(0.0, ca.sin(phi),  ca.cos(phi))
        )
        Rm = Z @ Y @ X

        # N matrix for ZYX convention
        N = ca.vertcat(
            ca.horzcat(1.0, 0.0, -ca.sin(theta)),
            ca.horzcat(0.0, ca.cos(phi), ca.cos(theta)*ca.sin(phi)),
            ca.horzcat(0.0, -ca.sin(phi), ca.cos(phi)*ca.cos(theta))
        )

        # Inertia calculations
        s = ca.sin(np.pi/4.0 - J/2.0)
        c = ca.cos(np.pi/4.0 - J/2.0)
        I_roll = Iyy_u * s**2.0 + Ixx_u * c**2.0 + Iyy_l * s**2.0 + Ixx_l * c**2.0
        I_pitch = Iyy_u * c**2.0 + Ixx_u * s**2.0 + Iyy_l * c**2.0 + Ixx_l * s**2.0
        I_yaw = Izz_u + Izz_l
        I = ca.diag(ca.vertcat(I_roll, I_pitch, I_yaw))


        # Forces and torques
        T = (F1 + F2 + F3 + F4) * ca.cos(delta) * 1.0
        Tx = (-F1 + F2 + F3 - F4) * (ca.cos(delta)*l*s + km*ca.sin(delta)*c) * 1.0
        Ty = (F1 - F2 + F3 - F4) * (ca.cos(delta)*l*c + km*ca.sin(delta)*s) * 1.0
        Tz = (-F1 - F2 + F3 + F4) * km * ca.cos(delta) * 1.0

        # State derivatives
        xdot = vx   
        ydot = vy
        zdot = vz
        r_ddot = linear_scale * (ca.vertcat(0.0, 0.0, g) + (Rm @ ca.vertcat(0.0, 0.0, -T)) / m)
        vxdot = r_ddot[0]
        vydot = r_ddot[1]
        vzdot = r_ddot[2]
        body_angle_rates = ca.solve(N, ca.vertcat(p, q, r))
        phidot = body_angle_rates[0]
        thetadot = body_angle_rates[1]
        psidot = body_angle_rates[2]
        body_angular_acc = ca.solve(I, ca.vertcat(Tx, Ty, Tz) - ca.cross(ca.vertcat(p, q, r), I @ ca.vertcat(p, q, r)))
        pdot = body_angular_acc[0]
        qdot = body_angular_acc[1]
        rdot = body_angular_acc[2]
        # Jdot = Jdot   
        Jddot = T_J / Izz_u

        dyn_f = ca.vertcat(xdot, ydot, zdot,
                           vxdot, vydot, vzdot,
                           phidot, thetadot, psidot,
                           pdot, qdot, rdot,
                           Jdot, Jddot)

        dyn_fun = ca.Function('f', [zeta_f, u], [dyn_f])

        return zeta_f, dyn_f, u, dyn_fun