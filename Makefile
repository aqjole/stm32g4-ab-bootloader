# G4Boot top-level build
#
#   make            bl + both app variants + packed images
#   make bl         bootloader only
#   make app_a      application linked for slot A
#   make app_b      application linked for slot B
#   make images     app_a.img + app_b.img (objcopy + pack)
#   make info       dump the packed headers
#   make sizes      arm-none-eabi-size for every artifact
#   make clean      remove all build output and images
#
# Homebrew's arm-none-eabi-gcc formula ships without newlib, so point at ARM's
# official toolchain (the gcc-arm-embedded cask) explicitly. Override on the
# command line if yours lives elsewhere:  make GCC_PATH=/path/to/bin
GCC_PATH ?= /Applications/ArmGNUToolchain/15.3.rel1/arm-none-eabi/bin

MAKE_APP = $(MAKE) -C app GCC_PATH=$(GCC_PATH)
MAKE_BL  = $(MAKE) -C bl  GCC_PATH=$(GCC_PATH)

OBJCOPY = $(GCC_PATH)/arm-none-eabi-objcopy
SIZE    = $(GCC_PATH)/arm-none-eabi-size
PACK    = python3 tools/pack.py

VERSION ?= 0.1.0

.PHONY: all bl app_a app_b images info sizes clean

all: images

bl:
	$(MAKE_BL) LDSCRIPT=bl.ld

# One source tree, two link addresses. Separate BUILD_DIR per variant so the
# object files never collide -- they are compiled identically but linked apart.
app_a:
	$(MAKE_APP) TARGET=app_a BUILD_DIR=build/a LDSCRIPT=app_a.ld

app_b:
	$(MAKE_APP) TARGET=app_b BUILD_DIR=build/b LDSCRIPT=app_b.ld

images: bl app_a app_b
	$(OBJCOPY) -O binary -S app/build/a/app_a.elf app/build/a/app_a.bin
	$(OBJCOPY) -O binary -S app/build/b/app_b.elf app/build/b/app_b.bin
	$(PACK) app/build/a/app_a.bin --slot A --version $(VERSION) -o app_a.img
	$(PACK) app/build/b/app_b.bin --slot B --version $(VERSION) -o app_b.img

info:
	$(PACK) --info app_a.img
	@echo
	$(PACK) --info app_b.img

sizes:
	$(SIZE) bl/build/bl.elf app/build/a/app_a.elf app/build/b/app_b.elf

clean:
	$(MAKE_BL) clean
	$(MAKE_APP) BUILD_DIR=build/a clean
	$(MAKE_APP) BUILD_DIR=build/b clean
	rm -f app_a.img app_b.img
