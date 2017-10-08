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

// parameter and all of its descendents are never modified (unless
// they are shared with a parameter that is modifiable).
#define in

// parameter is output only and will never be read
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
