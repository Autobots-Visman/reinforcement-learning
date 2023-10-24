# reinforcement learning

This is exploration into reinforcement learning with manipulators. 


## quickstart

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
