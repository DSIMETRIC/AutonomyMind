PLATFORM ?= linux/amd64

.PHONY: build test demo contract-check arm64-build

build:
	AUTONOMYMIND_PLATFORM=$(PLATFORM) docker compose build

test:
	AUTONOMYMIND_PLATFORM=$(PLATFORM) docker compose run --rm test

demo:
	AUTONOMYMIND_PLATFORM=$(PLATFORM) docker compose run --rm demo

contract-check:
	AUTONOMYMIND_PLATFORM=$(PLATFORM) docker compose run --rm contract-check

arm64-build:
	docker buildx build --platform linux/arm64 -t autonomymind/runtime:arm64 --load .
