# Robotics Suite Makefile

.PHONY: help install install-dev test lint format type-check clean docs run-demo

help:  ## Show this help message
	@echo "Available commands:"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "  \033[36m%-15s\033[0m %s\n", $$1, $$2}'

install:  ## Install the package
	pip install -e .

install-dev:  ## Install with development dependencies
	pip install -e ".[dev,docs]"

test:  ## Run comprehensive tests with 100% coverage target
	python run_tests.py

test-fast:  ## Run tests without coverage
	pytest tests/ -v

test-unit:  ## Run unit tests only
	pytest tests/ -v -m "not integration" --cov=src/robotics_suite --cov-report=term-missing

test-integration:  ## Run integration tests only
	pytest tests/ -v -m integration

test-coverage:  ## Run tests with detailed coverage
	pytest tests/ -v --cov=src/robotics_suite --cov-report=term-missing --cov-report=html --cov-report=xml --cov-fail-under=95

lint:  ## Run linting
	flake8 src/ tests/
	isort --check-only src/ tests/

format:  ## Format code
	black src/ tests/
	isort src/ tests/

type-check:  ## Run type checking
	mypy src/

quality:  ## Run all quality checks
	$(MAKE) lint
	$(MAKE) type-check
	$(MAKE) test

clean:  ## Clean build artifacts
	rm -rf build/
	rm -rf dist/
	rm -rf *.egg-info/
	rm -rf .pytest_cache/
	rm -rf htmlcov/
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete

docs:  ## Build documentation
	cd docs && make html

docs-serve:  ## Serve documentation locally
	cd docs/_build/html && python -m http.server 8000

run-demo:  ## Run basic demo
	robotics-demo --scenario pick_place --duration 30

run-production:  ## Run production line demo
	robotics-demo --scenario production_line --duration 60

validate:  ## Validate installation
	robotics-demo validate-setup

build:  ## Build package
	python -m build

publish-test:  ## Publish to test PyPI
	python -m twine upload --repository testpypi dist/*

publish:  ## Publish to PyPI
	python -m twine upload dist/*