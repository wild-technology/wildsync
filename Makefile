# ilxctl - Sony ILX-LR1 control panel
#
# Builds on macOS (Apple Silicon) and Linux (aarch64, e.g. Jetson Orin).
#
# The Camera Remote SDK ships as prebuilt shared libraries that resolve their
# siblings through an rpath relative to the loading binary, so the build stages
# the SDK next to the executable on both platforms:
#
#   macOS   dylibs, install name @rpath/..., link with -rpath @executable_path.
#           The SDK resolves its transport adapters through the app-bundle
#           convention, i.e. <exe_dir>/Contents/Frameworks/CrAdapter. Staging
#           them only in <exe_dir>/CrAdapter makes EnumCameraObjects fail with
#           0x8703 (CrError_Adaptor_Create), so both locations are populated -
#           which is exactly what Sony's own CMakeLists does for Apple builds.
#           The quarantine attribute must also be stripped or dyld refuses to
#           load the dylibs.
#
#   Linux   .so files with RUNPATH=$$ORIGIN baked in already; link with
#           -rpath '$$ORIGIN'. The adapters live in <exe_dir>/CrAdapter only -
#           the Contents/Frameworks duplication is an Apple-only convention, and
#           there is no quarantine attribute. Matches the NOT APPLE branch of
#           Sony's CMakeLists.

UNAME_S := $(shell uname -s)

BUILD    := build
BIN      := $(BUILD)/ilxctl
SRCS     := src/main.cpp src/camera.cpp
OBJS     := $(patsubst src/%.cpp,$(BUILD)/obj/%.o,$(SRCS))
DEPS     := $(OBJS:.o=.d)

CXXFLAGS := -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter -MMD -MP \
            -isystem include -isystem third_party -Isrc
LDFLAGS  := -Llib -lCr_Core -pthread

ifeq ($(UNAME_S),Darwin)
  CXX      ?= clang++
  DYLIB_EXT := dylib
  CXXFLAGS += -mmacosx-version-min=12.1
  LDFLAGS  += -Wl,-rpath,@executable_path
else
  CXX      ?= g++
  DYLIB_EXT := so
  # $$ORIGIN reaches make as $ORIGIN, which is what the dynamic linker wants.
  LDFLAGS  += -Wl,-rpath,'$$ORIGIN' -Wl,--disable-new-dtags
  # char is unsigned by default on ARM, so the SDK's CrInt8-backed enums (which
  # use -1 for Wide) will not compile without this. Sony's own CMakeLists sets
  # the same two flags for UNIX AND NOT APPLE.
  CXXFLAGS += -fsigned-char -fstack-protector-all
  # Tune for the host CPU (Cortex-A78AE on Orin). Silently skipped on
  # toolchains that reject it, so a generic build still works.
  CXXFLAGS += $(shell $(CXX) -mcpu=native -E -x c++ /dev/null >/dev/null 2>&1 \
                && echo -mcpu=native)
endif

SDK_LIBS := $(wildcard lib/*.$(DYLIB_EXT))
STAGED   := $(patsubst lib/%,$(BUILD)/%,$(SDK_LIBS)) $(BUILD)/.adapters

.PHONY: all run clean rebuild sdk sdk-linux sdk-check

all: sdk-check $(BIN) $(STAGED)

# The Sony SDK is not redistributed with this repo. Point CRSDK_DIR at an
# unpacked RemoteCli/SimpleCli package to populate include/CRSDK and lib/.
CRSDK_DIR ?= ../RemoteCli

sdk:
	@test -d "$(CRSDK_DIR)/app/CRSDK" || { \
	  echo "Not found: $(CRSDK_DIR)/app/CRSDK"; \
	  echo "Unpack the Camera Remote SDK and re-run:  make sdk CRSDK_DIR=/path/to/RemoteCli"; \
	  exit 1; }
	@mkdir -p include lib
	@cp -R "$(CRSDK_DIR)/app/CRSDK" include/
	@cp "$(CRSDK_DIR)"/external/crsdk/*.$(DYLIB_EXT) lib/
	@rm -rf lib/CrAdapter
	@cp -R "$(CRSDK_DIR)/external/crsdk/CrAdapter" lib/
ifeq ($(UNAME_S),Darwin)
	@xattr -dr com.apple.quarantine include lib 2>/dev/null || true
endif
	@echo "SDK staged from $(CRSDK_DIR)"

# Stage the LINUX aarch64 SDK into lib-linux/, for pushing to the Pi nodes.
#
# `sdk` above stages for THIS host - on a Mac that means .dylib - and that is
# the right thing for building ilxctl here. It is the wrong thing entirely for
# a node: deploy.sh rsyncs lib/ to the Pi, and macOS dylibs bricked a node
# build with a missing libCr_Core.so while looking like a successful provision
# (audit 2026-08-27), which is why deploy.sh now refuses to provision unless it
# can see a real .so.
#
# That refusal left no way to provision a node FROM the Mac at all - and since
# the Jetson was retired the Mac is the only host there is. This target closes
# that gap: it always copies .so, never $(DYLIB_EXT), and writes to a separate
# directory so the host's own lib/ is untouched and `make` here keeps working.
#
# Sony ships PLATFORM-SPECIFIC packages: point CRSDK_DIR at the unpacked
# Linux64ARMv8 one, not the Mac one. The headers are identical across both
# (verified byte-for-byte, 2026-08-28) so this target does not touch include/.
#
#   make sdk-linux CRSDK_DIR=/path/to/unpacked/CrSDK_..._Linux64ARMv8
#   deploy/deploy.sh provision cam3
sdk-linux:
	@test -d "$(CRSDK_DIR)/external/crsdk" || { \
	  echo "Not found: $(CRSDK_DIR)/external/crsdk"; \
	  echo "Unpack the LINUX ARMv8 Camera Remote SDK and re-run:"; \
	  echo "  make sdk-linux CRSDK_DIR=/path/to/unpacked/CrSDK_..._Linux64ARMv8"; \
	  exit 1; }
	@test -f "$(CRSDK_DIR)/external/crsdk/libCr_Core.so" || { \
	  echo "$(CRSDK_DIR) has no libCr_Core.so - that is the MAC or Windows"; \
	  echo "package. The nodes are aarch64 Linux; get the Linux64ARMv8 package."; \
	  exit 1; }
	@mkdir -p lib-linux
	# cp -R into an EXISTING directory recurses into it, so a second run
	# produced lib-linux/CrAdapter/CrAdapter and deploy.sh then rsynced the
	# nest to the node. Remove the destination first; the .so copies below
	# overwrite in place and are already idempotent.
	@cp "$(CRSDK_DIR)"/external/crsdk/*.so lib-linux/
	@rm -rf lib-linux/CrAdapter
	@cp -R "$(CRSDK_DIR)/external/crsdk/CrAdapter" lib-linux/
	@echo "Linux aarch64 SDK staged in lib-linux/ from $(CRSDK_DIR)"
	@echo "  $$(ls lib-linux/*.so lib-linux/CrAdapter/*.so 2>/dev/null | wc -l | tr -d ' ') shared objects"

sdk-check:
	@test -f include/CRSDK/CameraRemote_SDK.h && test -f lib/libCr_Core.$(DYLIB_EXT) || { \
	  echo "The Sony Camera Remote SDK is missing (it is not redistributed here)."; \
	  echo "Run:  make sdk CRSDK_DIR=/path/to/unpacked/RemoteCli"; \
	  exit 1; }

$(BUILD)/obj/%.o: src/%.cpp | $(BUILD)/obj
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(BIN): $(OBJS) | $(BUILD)
	$(CXX) $(OBJS) $(LDFLAGS) -o $@

$(BUILD) $(BUILD)/obj:
	@mkdir -p $@

# The SDK libraries must sit beside the binary (and on macOS be un-quarantined,
# or dyld refuses to load them).
$(BUILD)/%.$(DYLIB_EXT): lib/%.$(DYLIB_EXT) | $(BUILD)
	@cp $< $@
ifeq ($(UNAME_S),Darwin)
	@xattr -d com.apple.quarantine $@ 2>/dev/null || true
endif

$(BUILD)/.adapters: $(wildcard lib/CrAdapter/*.$(DYLIB_EXT)) | $(BUILD)
	@mkdir -p $(BUILD)/CrAdapter
	@cp lib/CrAdapter/*.$(DYLIB_EXT) $(BUILD)/CrAdapter/
ifeq ($(UNAME_S),Darwin)
	@mkdir -p $(BUILD)/Contents/Frameworks/CrAdapter
	@cp lib/CrAdapter/*.dylib $(BUILD)/Contents/Frameworks/CrAdapter/
	@xattr -dr com.apple.quarantine $(BUILD)/CrAdapter $(BUILD)/Contents 2>/dev/null || true
endif
	@touch $@

run: all
	@$(BIN)

clean:
	rm -rf $(BUILD)

rebuild: clean all

-include $(DEPS)
