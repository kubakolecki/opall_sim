import random
import numpy as np

def deg_to_rad(deg):
    rad = 0.01745329251994329576923690768489*deg
    return rad


def gaussian_noise_with_limit(sigma):
    #this function allows to avoid randomly sampling of large outliers
    limit = 7
    noise = random.gauss(0.0, sigma)
    if noise > limit*sigma:
        noise = limit
    if noise < -limit*sigma:
        noise = -limit
    return noise

lie_alg_basis_x = np.array([[0,0,0],[0,0,-1],[0,1,0]])

lie_alg_basis_y = np.array([[0,0,1],[0,0,0],[-1,0,0]])

lie_alg_basis_z = np.array([[0,-1,0],[1,0,0],[0,0,0]])

def generate_noise_on_so3(sigma):
    noise_rotation = np.eye(3)
    ex = gaussian_noise_with_limit(sigma)
    ey = gaussian_noise_with_limit(sigma)
    ez = gaussian_noise_with_limit(sigma)
    e = np.array([ex,ey,ez])
    theta = np.linalg.norm(e)
    e *= (1.0/theta)
    eR = e[0]*lie_alg_basis_x + e[1]*lie_alg_basis_y + e[2]*lie_alg_basis_z
    noise_rotation += np.sin(theta)*eR + (1-np.cos(theta))*(eR@eR)
    return noise_rotation


