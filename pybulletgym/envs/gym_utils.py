import inspect
import os

from pybulletgym.envs.roboschool.robots.robot_bases import BodyPart

currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(0, parentdir)
import pybullet_data


def get_cube(p, x, y, z):
    body = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "cube_small.urdf"), [x, y, z])
    p.changeDynamics(body, -1, mass=1.2)  # match Roboschool
    part_name, _ = p.getBodyInfo(body)
    part_name = part_name.decode("utf8")
    bodies = [body]
    return BodyPart(p, part_name, bodies, 0, -1)


def get_sphere(p, x, y, z):
    body = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "sphere2red_nocol.urdf"), [x, y, z])
    part_name, _ = p.getBodyInfo(body)
    part_name = part_name.decode("utf8")
    bodies = [body]
    return BodyPart(p, part_name, bodies, 0, -1)
