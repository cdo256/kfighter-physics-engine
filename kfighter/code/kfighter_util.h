#if !defined(KFIGHTER_UTIL_H)

#include "kfighter_global.h"
#include "kfighter.h"

global const u32 initialSeed = 1;

internal void seedRandomNumberGenerator(modified_(randomSeed) GameState* state);

//NOTE: Generates between 0 and 1
internal f32 rand(modified_(randomSeed) GameState* state);

//NOTE: Decides if the button was depressed or released, doesn't
//capture single frame presses
internal bool wasDepressed(GameButtonState button);
internal bool wasReleased(GameButtonState button);
internal bool wasTapped(GameButtonState button);

internal void makeWalls(modified GameState* state, in GameOffscreenBuffer* buffer);

#define KFIGHTER_UTIL_H
#endif
