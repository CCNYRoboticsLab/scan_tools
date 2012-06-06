all: installed

BUILD_DIR = build
SOURCE_DIR = $(BUILD_DIR)/csm-git

########
## Install from git
#GIT_URL = http://github.com/AndreaCensi/csm.git
#GIT_REVISION = 23703a998fff57250322
#include $(shell rospack find mk)/git_checkout.mk
## Install from tarball
TARBALL     = build/csm-git-23703a998fff57250322.tar.gz
TARBALL_URL = https://code.ros.org/svn/release/download/thirdparty/csm-git-23703a998fff57250322.tar.gz
UNPACK_CMD  = tar zxf
MD5SUM_FILE = csm-git-23703a998fff57250322.tar.gz.md5sum
include $(shell rospack find mk)/download_unpack_build.mk
########

patch:
ifneq (,$(wildcard $(BUILD_DIR)/.patched))
	@echo Patching CSM
	cd $(SOURCE_DIR) && patch -p0 < ../../patch_a
	cd $(SOURCE_DIR) && patch -p0 < ../../patch_b
	cd $(SOURCE_DIR) && patch -p0 < ../../patch_c
	touch $(BUILD_DIR)/.patched
else
	@echo CSM previously patched, use 'make wipe' to completely remove build directory.
endif

installed: $(SOURCE_DIR)/unpacked patch
	cd $(SOURCE_DIR) && mkdir -p deploy
	cd $(SOURCE_DIR) && cmake -DCMAKE_INSTALL_PREFIX:PATH=$(CURDIR) -DCMAKE_C_FLAGS:STRING="-fPIC"
	cd $(SOURCE_DIR) && make
	cd $(SOURCE_DIR) && make install
	touch installed

clean:
	-cd $(SOURCE_DIR) && make clean
	rm -rf $(CURDIR)/lib $(CURDIR)/include $(CURDIR)/bin installed 

wipe: clean
	rm -rf $(BUILD_DIR)

