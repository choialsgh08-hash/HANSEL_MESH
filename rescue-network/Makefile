# Convenience targets for local development.
# Windows users can run the equivalent commands from the README instead.

PY ?= python3
VENV ?= .venv
BIN := $(VENV)/bin

.PHONY: help venv install test lint fmt run-field clean

help:
	@echo "Targets:"
	@echo "  venv       create a virtualenv in $(VENV)"
	@echo "  install    install the package with dev extras"
	@echo "  test       run pytest"
	@echo "  lint       run ruff check"
	@echo "  fmt        run ruff format"
	@echo "  run-field  start the field web server (port 8000, dev)"

venv:
	$(PY) -m venv $(VENV)

install:
	$(BIN)/python -m pip install --upgrade pip
	$(BIN)/pip install -e ".[dev]"

test:
	$(BIN)/pytest

lint:
	$(BIN)/ruff check .

fmt:
	$(BIN)/ruff format .

# Dev uses port 8000; production field node binds :80 (see systemd, Phase 3).
run-field:
	$(BIN)/uvicorn rescue_network.field_app:app --reload --port 8000

clean:
	rm -rf $(VENV) data .pytest_cache .ruff_cache
