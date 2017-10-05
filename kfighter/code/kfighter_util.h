#if !defined(KFIGHTER_UTIL_H)
/* ========================================================================
   $File: $
   $Date: $
   $Revision: $
   $Creator: Christopher O'Donnell $
   $Notice: (C) Copyright 2017 by Dipole Limited. All Rights Reserved. $
   ======================================================================== */

#include "kfighter_global.h"
#include "kfighter.h"

global const u32 initialSeed = 1;

internal void seedRandomNumberGenerator(GameState* state);

//NOTE: Generates between 0 and 1
internal f32 rand(GameState* state);

//NOTE: Decides if the button was depressed or released, doesn't
//capture single frame presses
internal bool wasDepressed(GameButtonState button);
internal bool wasReleased(GameButtonState button);
internal bool wasTapped(GameButtonState button);

internal void makeWalls(GameState* state, GameOffscreenBuffer* buffer);

#define KFIGHTER_UTIL_H
#endif
