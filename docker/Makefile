.PHONY: build run up shell stop start down clean profiles check-profile check-env

#list available profiles
profiles:
	@echo "Available profiles:"
	@find . -maxdepth 1 -type d ! -name "." | while read dir; do \
		if [ -f "$$dir/docker-compose.yml" ]; then \
			echo "  $$(basename $$dir)"; \
		fi; \
	done

#check profile for selecting docker-compose file
check-profile:
	@if [ -z "$(PROFILE)" ]; then \
		echo "PROFILE not set."; \
		echo "Please specify which Docker profile to use. For example:"; \
		echo "  make build PROFILE=minimal"; \
		exit 1; \
	fi

#check if .env file exists and if DATASETS_PATH is set
check-env:
	@if [ ! -f .env ]; then \
		echo "Warning: .env file not found. Creating one with default DATASETS_PATH."; \
		echo "DATASETS_PATH=/root/empty_data" > .env; \
	elif ! grep -q '^DATASETS_PATH=' .env; then \
		echo "DATASETS_PATH not set in .env. Adding default fallback."; \
		echo "DATASETS_PATH=/root/empty_data" >> .env; \
	fi
	@grep '^DATASETS_PATH=' .env

DOCKER_DIR := $(PROFILE)
SERVICE_NAME := hydra-$(PROFILE)

#build the selected image
build: check-profile check-env
	docker compose --env-file .env -f $(DOCKER_DIR)/docker-compose.yml build

#start one-time interactive container, auto-removed when exit
run: check-profile check-env
	docker compose --env-file .env -f $(DOCKER_DIR)/docker-compose.yml run --rm $(SERVICE_NAME) bash

#start the container in detached mode to keep container running
up: check-profile check-env
	docker compose --env-file .env -f $(DOCKER_DIR)/docker-compose.yml up -d

#attach a shell to the running container (must already be ran via up)
shell: check-profile check-env
	docker compose --env-file .env -f $(DOCKER_DIR)/docker-compose.yml exec $(SERVICE_NAME) bash

#stop the running the container
stop: check-profile check-env
	docker compose --env-file .env -f $(DOCKER_DIR)/docker-compose.yml stop

#restart a container that was stopped
start: check-profile check-env
	docker compose --env-file .env -f $(DOCKER_DIR)/docker-compose.yml start

#stop and remove everything created by up
down: check-profile check-env
	docker compose --env-file .env -f $(DOCKER_DIR)/docker-compose.yml down

#remove all unused containers
clean: check-profile check-env
	docker container prune -f
	docker image prune -f
