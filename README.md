# reinforcement learning

This is exploration into reinforcement learning with manipulators.

- https://gymnasium.farama.org/environments/mujoco/
- https://robotics.farama.org/
  - This isn't compatible with 2.3.7 which is the latest version of mujoco at time of writing.

## quickstart

You will need Python 3.11 installed.
Install poetry and then run an example to make sure everything is working as expected.

```bash
pip install poetry

poetry install
poetry run shell

python -m src.examples.fetch_reach
```
