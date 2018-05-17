PyBullet Gymperium
-----------

*PyBullet Gymperium is an open-source implementation of the OpenAI Gym MuJoCo environments for use with the OpenAI Gym Reinforcement Learning Research Platform in support of open research.*

OpenAI gym is currently one of the most widely used toolkit for developing and comparing reinforcement learning algorithms. Unfortunately, for several challenging continuous control environments it requires the user to install MuJoCo, a commercial physics engine which requires a license to run for longer than 30 days. Such a commercial barrier hinders open research, especially in the perspective that other appropriate physics engines exist. This repository provides alternative implementations of the original MuJoCo environments which can be used for free. The environments have been reimplemented using [BulletPhysics'](https://github.com/bulletphysics/bullet3) python wrapper pybullet, such that they seamlessly integrate into the OpenAI gym framework. In order to show the usability of the new environments, several RL agents from the [Keras-RL](http://keras-rl.readthedocs.io/en/latest/agents/overview/) are configured to be trainable out of the box. To further simplify the training of agents, a Trainer class was implemented which helps to capture commandline arguments in a unified fashion. The Trainer provides a set of standard arguments, but additional arguments can be defined by the agent and the environment to enable the researcher to provide special parameters to either one.

<!--
!PLEASE REVIEW ALL THIS INFORMATION BEFORE PUBLISHING!


[See What's New section below](#What's New)

## Basics

There are two basic concepts in reinforcement learning: the
environment (namely, the outside world) and the agent (namely, the
algorithm you are writing). The agent sends `actions` to the
environment, and the environment replies with `observations` and
`rewards` (that is, a score).

The core `gym` interface is `Env <https://github.com/openai/gym/blob/master/gym/core.py>`_, which is
the unified environment interface. There is no interface for agents;
that part is left to you. The following are the ``Env`` methods you
should know:

- `reset(self)`: Reset the environment's state. Returns `observation`.
- `step(self, action)`: Step the environment by one timestep. Returns `observation`, `reward`, `done`, `info`.
- `render(self, mode='human', close=False)`: Render one frame of the environment. The default mode will do something human friendly, such as pop up a window. Passing the `close` flag signals the renderer to close any such windows.

==Add concept of scene and robot as subconcept of environment

## Installation

You can perform a minimal install of ``gym`` with:

```bash
	  git clone https://github.com/openai/gym.git
	  cd gym
	  pip install -e .
```

If you prefer, you can do a minimal install of the packaged version directly from PyPI:

```bash
	  pip install gym
```


## Supported systems

We currently support Linux, Windows and OS X running Python 2.7 or 3.5.

## Pip version

To run ``pip install -e '.[all]'``, you'll need a semi-recent pip.
Please make sure your pip is at least at version ``1.5.0``. You can
upgrade using the following: ``pip install --ignore-installed
pip``. Alternatively, you can open `setup.py
<https://github.com/openai/gym/blob/master/setup.py>`_ and
install the dependencies by hand.

Rendering on a server
---------------------

If you're trying to render video on a server, you'll need to connect a
fake display. The easiest way to do this is by running under
``xvfb-run`` (on Ubuntu, install the ``xvfb`` package):

```bash
     xvfb-run -s "-screen 0 1400x900x24" bash
```

Installing dependencies for specific environments
-------------------------------------------------

If you'd like to install the dependencies for only specific
environments, see `setup.py
<https://github.com/openai/gym/blob/master/setup.py>`_. We
maintain the lists of dependencies on a per-environment group basis.

## Agents

==Add some agents

## Environments

The code for each environment group is housed in its own subdirectory
`gym/envs
<https://github.com/openai/gym/blob/master/gym/envs>`_. The
specification of each task is in `gym/envs/__init__.py
<https://github.com/openai/gym/blob/master/gym/envs/__init__.py>`_. It's
worth browsing through both.

## Examples

See the 'examples' directory.

- Run [File here](link here) to run an actual learning agent on the something environment.

##Add all examples

-->

## What's new

* 2018-01-09 Pybullet-gym is born.

## Roadmap
1. Replicate Gym MuJoCo environments.
2. Implement DeepMind Control Suite gyms.
3. Explore other types of gyms/multi agent gyms.

