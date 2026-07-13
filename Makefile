.PHONY: all clean flash

all: app

app: app/out/merged.hex

app/out/merged.hex:
	@echo "Building: $@"; \
	ZEPHYR_BASE=${ZEPHYR_NCS_ROOT}/zephyr west build -b xm126/nrf52840 app --pristine --build-dir app/out -- -DBOARD_ROOT=$(shell pwd)/acconeer

flash: app/out/merged.hex
	@echo "Flashing"; \
	nrfjprog -f nrf52 --program $< --sectorerase --verify --reset

clean:
	rm -rf app/out
