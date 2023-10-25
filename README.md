# Reinforcement Learning

This is exploration into reinforcement learning with manipulators. We're trying to build a modular environment to train RL-Based agents by providing a standard REST Based Interface to access multiple simulations environment based of Handy Maniupulators.

On the otherside, we're tryint to design an agent that could learn and benefit from this environment setup.

![RL_Architecture](/doc/images/rl_idea_architecture.png "MarineGEO logo")


## Quickstart

You will need Python 3.11 installed.
Install poetry and then run an example to make sure everything is working as expected.

```bash
pip install poetry

poetry install
poetry run shell

python -m src.examples.fetch_reach
```

To run the gazebo simulation, run the following commands which require docker.

```bash
# ensure the X server is open to connections from docker
xhost +local:docker

# build the container images for gazebo and ros
make build

# run everything
make up

# You can enter a running container by running the following command
docker exec -it [docker_id] /bin/bash

# You can run a new docker image (e.g. to use for development) for the base image using
docker run -it [image_id] /bin/bash
```

## generating docs

We use pandoc to generate a report from markdown.

```bash
make report
```

## demos

- 2023-10-03: FetchReach-v2 with random actions: https://youtu.be/16np2Y5eIGA

## notes and reading

- https://gymnasium.farama.org/environments/mujoco/
- https://robotics.farama.org/
  - This isn't compatible with 2.3.7 which is the latest version of mujoco at time of writing.
