# reinforcement learning

This is exploration into reinforcement learning with manipulators.

## quickstart

To run the gazebo simulation, run the following commands which require docker.

```bash
# ensure the X server is open to connections from docker
xhost +local:docker

# build the container images for gazebo and ros
make build

# run everything
make up
```

Go to the [agent](agent/README.md) directory for more information on the agent.

## installing new dependencies

To add a new python dependency to the ros packages, add a `requirements.in` to the package root.
Then add a reference to it from the top-level `requirements.in`.
Then compile the dependencies using `pip-compile` via `pip-tools`.

```bash
pip install pip-tools
pip-compile

# or via docker
docker compose run base pip-compile
sudo chown -R $(id -u):$(id -g) requirements.txt
```

Rebuild the docker images to pull in the new dependencies.

## notes on wsl2

Using host networking mode in docker on wsl2 will end up using the dockerd networking stack instead of the one inside of your wsl2 environment.
We need to use host networking for ROS to work properly, due to the dynamic allocation of ports and the fixed ip addresses of various nodes.
This conflicts the idea of running a web service, since the dockerd wsl2 environment is firewalled from the the rest of the network.
To get around this, we can use tailscale to create a vpn tunnel to the host machine.

```bash
docker run --name=tailscaled -v /var/lib:/var/lib -v /dev/net/tun:/dev/net/tun --network=host --cap-add=NET_ADMIN --cap-add=NET_RAW tailscale/tailscale
```

Authenticate, and also authenticate on the host machine you want to interact with the service from.

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
