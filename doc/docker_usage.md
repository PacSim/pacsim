# How to use

## Docker Compose

The simulator is inside a docker container, which by running it, the simulator will also be launched. To do this, run the following command on your terminal:

```bash
docker compose up
```

## Code Development - Dev Container

In case you want to do some work inside the container, by using the Dev Containers extension in VSCode the process becomes very simple, since there is a .devcontainer which contains the configuration for a development container for that purpose.

In order to run this, first you need to install some extensions. You can get them through here [Remote Development Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack).
Then you need to create some cache folders. This can be done by running the following commands:

```bash
mkdir cache
mkdir cache/iron-ros-base-jammy
mkdir cache/iron-ros-base-jammy/build
mkdir cache/iron-ros-base-jammy/install
mkdir cache/iron-ros-base-jammy/log
```
Finally do Ctrl + Shift + P and choose the option:

- Dev Containers: Rebuild and Reopen in container