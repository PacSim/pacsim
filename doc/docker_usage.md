# How to use

## Docker Compose

The simulator is inside a docker container, which by running it, the simulator will also be launched. To do this, run the following command on your terminal:

```bash
docker compose up
```

The *docker-compose.yml* file uses the same Docker image as the development container (Image in [.devcontainer folder](../.devcontainer)) and executes the [entrypoint.sh script](./entrypoint.sh) at startup.

## Code Development - Dev Container

In case you want to do some work inside the container, by using the Dev Containers extension in VSCode the process becomes very simple, since there is a .devcontainer which contains the configuration for a development container for that purpose.

To run this, first you need to install some extensions. You can get them through here [Remote Development Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack).
Then you need to create some cache folders. This can be done by running the following commands:

```bash
./environment_setup.sh
```
Finally do Ctrl + Shift + P and choose the option:

- Dev Containers: Rebuild and Reopen in container

## General Notes

For the simulator to communicate with your system, make sure to set the ROS Domain ID to the same value in both systems:
```sh
export ROS_DOMAIN_ID=42
```
