.DEFAULT_GOAL := build

.PHONY: build test

.PHONY: no_targets list

SHELL = /usr/bin/env bash

#############
# Variables # 
#############
# Git
CI_BUILD_CONCURRECY ?= 40
CI_COMMIT_SHORT_SHA ?= no_sha
CI_COMMIT_REF_NAME ?= master
CI_PROJECT_DIR ?= $(shell pwd)
GIT_CLONE_COMMAND ?= git clone --recurse-submodules -j$(CI_BUILD_CONCURRECY)

# Athena athena_vision build variables
ATHENA_REPOS_URL ?= https://partner-gitlab.mioffice.cn/cyberdog/athena_cyberdog.git

CYBERDOG_PATH ?= /opt/ros2/cyberdog
CROSS_ROOT_PATH ?= /mnt/sdcard
CROSS_HOME_PATH ?= $(CROSS_ROOT_PATH)/home
CROSS_SETUP_BASH_FILE ?= $(CROSS_ROOT_PATH)/opt/ros2/foxy/local_setup.bash
CROSS_CYBERDOG_PATH ?= $(CROSS_ROOT_PATH)$(CYBERDOG_PATH)
CROSS_BUILD_FLAG ?= --cmake-force-configure --cmake-args -DCMAKE_TOOLCHAIN_FILE=/home/builder/toolchainfile.cmake --merge-install --event-handlers console_cohesion+ --install-base $(INSTALL_BASE) --parallel-workers $(CI_BUILD_CONCURRECY) --packages-up-to $(PACKAGE_NAME)
CROSS_TEST_FLAG ?= --merge-install --event-handlers console_cohesion+ --return-code-on-test-failure --install-base $(INSTALL_BASE) --parallel-workers $(CI_BUILD_CONCURRECY) --packages-select $(TEST_NAMES)

REPO_NAME := $(lastword $(subst /, ,$(CI_PROJECT_DIR)))
PACKAGE_NAME ?= athena_automation
TEST_NAMES ?= $(shell cd $(CI_PROJECT_DIR) && colcon list -n |grep -Ev "chassis_node|dm_planner|realsense2_description"|xargs)
ATHENA_REPOS_NAME ?= src
INSTALL_BASE ?= $(CROSS_HOME_PATH)/$(REPO_NAME)/athena_ros2_deb/src$(CYBERDOG_PATH)
CLEAN_FILES ?= $(ATHENA_REPOS_NAME) athena_ros2_deb/src$(CYBERDOG_PATH)/*
BUILD_FILE_NAME ?= $(REPO_NAME)_athena_ros2_deb_src-$(CI_COMMIT_SHORT_SHA).tar
BUILD_TMP_FILE_LIST ?= build src $(REPO_NAME) $(CROSS_ROOT_PATH)/opt/ros2
BUILD_TMP_FILE_COPY_FLAGS ?= mnt/* /mnt
# FDS variables
FDS_BUILD_TMP_FILE_DIR ?= $(FDS_URL_PREFIX)/$(REPO_NAME)/$(CI_COMMIT_REF_NAME)/build_temp
FDS_BUILD_TMP_FILE_NAME ?= $(REPO_NAME)_tmp-$(CI_COMMIT_SHORT_SHA).tar

#################
# BUILD TARGET  #
#################
build: touch-files build-files

touch-files:
	@cp -arp $(CI_PROJECT_DIR) $(CROSS_HOME_PATH) && \
		mkdir -p $(CROSS_HOME_PATH)/build && \
		cd $(CROSS_HOME_PATH) && \
		rm -rf $(CLEAN_FILES) && \
		$(GIT_CLONE_COMMAND) $(ATHENA_REPOS_URL) $(ATHENA_REPOS_NAME) && \
		find $(ATHENA_REPOS_NAME) -name $(REPO_NAME)| xargs -i -t rm {} -rf

build-files:
	@cd $(CROSS_HOME_PATH) && \
		source $(CROSS_SETUP_BASH_FILE) && \
		colcon build $(CROSS_BUILD_FLAG)

upload-files:
	@cd $(CROSS_HOME_PATH)/$(REPO_NAME) && \
		tar cf $(BUILD_FILE_NAME) athena_ros2_deb/src && \
		$(FDS_COMMAND_UPLOAD) $(BUILD_FILE_NAME) $(FDS_URL_PREFIX)/$(REPO_NAME)/$(CI_COMMIT_REF_NAME)/$(BUILD_FILE_NAME)

#################                                                      
# TEST TARGET   #
#################
test:
	@cd $(CROSS_HOME_PATH) && \
		source $(CROSS_SETUP_BASH_FILE) && \
		colcon test $(CROSS_TEST_FLAG)


################
# Minor Targets#
################
no_targets:
list:
	@sh -c "$(MAKE) -p no_targets | awk -F':' '/^[a-zA-Z0-9][^\$$#\/\\t=]*:([^=]|$$)/ {split(\$$1,A,/ /);for(i in A)print A[i]}' | grep -v '__\$$' \
		| grep -v 'make'| grep -v 'list'| grep -v 'no_targets' |grep -v 'Makefile' | sort | uniq"

tmp-upload:
	@cd $(CROSS_HOME_PATH) && \
		tar cf $(FDS_BUILD_TMP_FILE_NAME) $(BUILD_TMP_FILE_LIST) && \
		$(FDS_COMMAND_UPLOAD) $(FDS_BUILD_TMP_FILE_NAME) $(FDS_BUILD_TMP_FILE_DIR)/$(FDS_BUILD_TMP_FILE_NAME)

tmp-download:
	@mkdir -p $(CROSS_HOME_PATH) $(CYBERDOG_PATH) && \
		cd $(CROSS_HOME_PATH) && \
		$(FDS_COMMAND_DOWNLOAD) $(FDS_BUILD_TMP_FILE_DIR)/$(FDS_BUILD_TMP_FILE_NAME) $(FDS_BUILD_TMP_FILE_NAME) && \
		tar xf $(FDS_BUILD_TMP_FILE_NAME) && \
		cp -r $(BUILD_TMP_FILE_COPY_FLAGS)
