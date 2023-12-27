# zephyr-trinamic
Zephyr OS support for Trinamic TMC drivers

## Initialize workspace

### 1. Production module

```
west init -m https://github.com/cooked/zephyr-trinamic zephyr-trinamic
# update modules
cd zephyr-trinamic
west update
```

### 2. Local / Development

A manifest folder is available, on top of the root west.yml, for the brave
developer that wants to have the current repo folder (and not the parent) set up
as the west workspace.
This manifest pulls in the Zephyr source code and place it in the "rtos" folder
instead of the default "zephyr" folder, to avoid overlapping with the
existing zephyr folder that contains the module definition (module.yml).

```
cd <this repo folder>
west init -l manifest-devel
west update
```

## Build & Run
The available samples can be built by running:

```
west build -b $BOARD samples/tmc-spi
```

Once it's built you can flash it by running:

```
west flash
```
