.PHONY: build build-debug

build:
	@echo "Building ..."
	@python3 build.py -g make

build-debug:
	@echo "Building for debug ..."
	@python3 build.py -g make -d yes
