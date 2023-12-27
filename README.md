# zephyr-trinamic
Zephyr OS support for Trinamic TMC drivers

## Initialize workspace

```
west init -m https://github.com/cooked/zephyr-trinamic --mr main zephyr-trinamic
# update modules
cd zephyr-trinamic
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
