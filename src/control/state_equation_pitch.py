#!/usr/bin/env python

import sympy as sp
import numpy as np
import control
from EoM import *
from sympy.physics.mechanics import *
import pylab as pl

def Cal_Pitch_SS():
    
    I_1, I_w, m_1, m_w, r, l, T_w, g= sp.symbols('I_1, I_w, m_1, m_w, r, l, T_w, g')
    theta_1, theta_w = dynamicsymbols('theta_1, theta_w')

    theta_wd = theta_w.diff()
    theta_wdd = theta_wd.diff()
    theta_1d = theta_1.diff()
    theta_1dd = theta_1d.diff()

    q = sp.Matrix([[theta_w], [theta_1]])
    qd = q.diff()
    qdd = qd.diff()

    u = sp.Matrix([T_w])

    tau = sp.Matrix([[((m_w+m_1)*r**2 + I_w)*theta_wdd + m_1*r*l*theta_1dd*sp.cos(theta_1) - m_1*r*l*theta_1d**2 * sp.sin(theta_1) - T_w],
                    [(I_1 + m_1*l**2)*theta_1dd + m_1*r*l*sp.cos(theta_1)*theta_wdd - m_1*g*l*sp.sin(theta_1) + T_w]])
    
    eq_point = {sp.sin(theta_1):theta_1, sp.cos(theta_1):1, theta_1d**2:0}
    tau_eq = sp.simplify(tau.subs(eq_point))
    
    Ml, Cl, Gl, Wl = get_EoM_from_T(tau_eq,qdd,g,u)
    param = {I_1: 1.5684639621472638, I_w:0.002997277,  m_1:21.731,   m_w:2.292, r:0.069, l: 0.5920165410935432,  g:9.81}

    Mlp = msubs(Ml, param)
    Clp = msubs(Cl, param)
    Glp = msubs(Gl, param)
    Wlp = msubs(Wl, param)
    
    Mlp_inv = Mlp.inv()
    qdd_rhs_A = Mlp_inv*(-Clp -Glp)
    qdd_rhs_B = Mlp_inv*Wlp*u
    X = q.col_join(qd)
    Xd_A = qd.col_join(qdd_rhs_A)
    Xd_B = qd.col_join(qdd_rhs_B)
    U = u
    A = Xd_A.jacobian(X)
    B = Xd_B.jacobian(U)
    C = X.jacobian(X)
    D = X.jacobian(U)

    return A, B, C, D
