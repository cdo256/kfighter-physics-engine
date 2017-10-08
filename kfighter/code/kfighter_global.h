#if !defined(KFIGHTER_GLOBAL_H)
/* ========================================================================
   $File: $
   $Date: $
   $Revision: $
   $Creator: Christopher O'Donnell $
   $Notice: (C) Copyright 2017 by Dipole Limited. All Rights Reserved. $
   ======================================================================== */

#define internal static
#define local_persist static
#define global static
#define static_storage static

// parameter and all of its descendents are never modified (unless
// they are shared with a parameter that is modifiable).
#define in

//TODO: Do we need partial_out or complete_out to show how much of the
//struct is filled? (eg. setPhysicsConstraints)

// parameter is output only 
#define out

// parameter may both be read and written to (including all descendents)
#define modified

// parameter is pointer to struct and an object pointed to from that
// struct is modifed
#define modified_descendent

// parameter is a struct pointer and this list indicates what is modified
#define modified_(...)

#define KFIGHTER_GLOBAL_H
#endif
