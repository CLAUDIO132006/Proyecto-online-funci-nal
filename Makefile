# Slendytubbies PS2 - single-file build (src/main.c + embedded assets)
# Requires: PS2DEV / PS2SDK / GSKIT environment variables (see docs/BUILDING.md)
# and the generated/ directory produced by tools/export_*.py.
# Usage:  make            (build/SLENDY.ELF)
#         make FAST=1     (adds -ffast-math, like the original render.c had)
#         make clean

EE_BIN = build/SLENDY.ELF
EE_OBJS = build/main.o build/assets.o
EE_INCS = -Isrc -I$(GSKIT)/include
EE_CFLAGS = -std=gnu99 -O3 -fno-math-errno -fno-trapping-math -ffunction-sections -fdata-sections -Wextra
ifdef FAST
EE_CFLAGS += -ffast-math
endif
EE_LDFLAGS = -L$(GSKIT)/lib -Wl,--gc-sections -Wl,-Map,build/SLENDY.map
EE_LIBS = -lgskit -ldmakit -lpad -laudsrv -lps2ip_nm -lnetman -lpatches -lm

GENERATED_HDRS = generated/noonoo.h generated/menu/menu_layout.h generated/scene_visuals.h \
	generated/scene_audio.h generated/fidelity.h generated/victory/victory_layout.h generated/player_walk.h
TEST_HDRS = tests/menu_route.h tests/benchmark.h tests/walk_route.h

all: $(EE_BIN)

build/main.o: src/main.c $(GENERATED_HDRS) $(TEST_HDRS) Makefile
	mkdir -p build
	$(EE_CC) $(EE_CFLAGS) $(EE_INCS) -c $< -o $@

build/assets.S: generated/world.bin generated/menu/world.bin generated/victory/world.bin generated/player_walk.bin
	mkdir -p build
	printf '.section .rodata\n.balign 128\n.global world_start, world_end\nworld_start:\n.incbin "generated/world.bin"\nworld_end:\n' > $@
	printf '.balign 128\n.global menu_world_start,menu_world_end\nmenu_world_start:\n.incbin "generated/menu/world.bin"\nmenu_world_end:\n' >> $@
	printf '.balign 128\n.global victory_world_start,victory_world_end\nvictory_world_start:\n.incbin "generated/victory/world.bin"\nvictory_world_end:\n' >> $@
	printf '.balign 128\n.global player_walk_start,player_walk_end\nplayer_walk_start:\n.incbin "generated/player_walk.bin"\nplayer_walk_end:\n' >> $@
	printf '.balign 64\n.global sio2_start,sio2_end\nsio2_start:\n.incbin "$(PS2SDK)/iop/irx/freesio2.irx"\nsio2_end:\n' >> $@
	printf '.balign 64\n.global pad_start,pad_end\npad_start:\n.incbin "$(PS2SDK)/iop/irx/freepad.irx"\npad_end:\n' >> $@
	printf '.balign 64\n.global audsrv_start,audsrv_end\naudsrv_start:\n.incbin "$(PS2SDK)/iop/irx/audsrv.irx"\naudsrv_end:\n' >> $@
	printf '.balign 64\n.global dev9_start,dev9_end\ndev9_start:\n.incbin "$(PS2SDK)/iop/irx/ps2dev9.irx"\ndev9_end:\n' >> $@
	printf '.balign 64\n.global netman_start,netman_end\nnetman_start:\n.incbin "$(PS2SDK)/iop/irx/netman.irx"\nnetman_end:\n' >> $@
	printf '.balign 64\n.global smap_start,smap_end\nsmap_start:\n.incbin "$(PS2SDK)/iop/irx/smap.irx"\nsmap_end:\n' >> $@
	printf '.balign 64\n.global ps2ipnm_start,ps2ipnm_end\nps2ipnm_start:\n.incbin "$(PS2SDK)/iop/irx/ps2ip-nm.irx"\nps2ipnm_end:\n' >> $@

build/assets.o: build/assets.S
	$(EE_CC) $(EE_CFLAGS) $(EE_INCS) -c $< -o $@

clean:
	rm -rf build

include $(PS2SDK)/samples/Makefile.pref
include $(PS2SDK)/samples/Makefile.eeglobal
