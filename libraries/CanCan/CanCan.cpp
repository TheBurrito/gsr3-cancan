#include "CanCan.h"
#include "gsr_lcd.h"

//namespace arena {

Bounds arenas[arenaCount];
Bounds arena;

void setArena(int i, double buffer) {
	float arena_w, arena_l;
	
	switch (i) {
		case 0:
		case 1:
			arena_w = 7;
			arena_l = 10;
			break;
			
		case 2:
		case 3:
			arena_w = 4;
			arena_l = 8;
			break;
	}
    
    arena.width = arena_w;
    arena.length = arena_l;
    
    arena.max.x = arena_l * 12 * 2.54 - buffer - 13;
    arena.max.y = arena_w / 2 * 12 * 2.54 - buffer;
    
    arena.min.x = 0 + buffer;
    arena.min.y = -arena_w / 2 * 12 * 2.54 + buffer;
    
    arena.goal.x = arena.max.x + 5;
    arena.goal.y = 0;
}

//}
