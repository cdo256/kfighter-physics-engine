/* ========================================================================
   $File: $
   $Date: $
   $Revision: $
   $Creator: Christopher O'Donnell $
   $Notice: (C) Copyright 2017 by Dipole Limited. All Rights Reserved. $
   ======================================================================== */


internal void renderBackground(GameState* state, GameOffscreenBuffer* buffer) {
    for (u32 y = 0; y < buffer->height; y++) {
        u8* row = (u8*)buffer->bitmapMemory + buffer->pitch * y;
        for (u32 x = 0; x < buffer->width; x++) {
            u8* pixel = row + buffer->bytesPerPixel * x;
            *((u32*)pixel) = state->backgroundColour;
        }
    }
}

internal void renderPoint(GameOffscreenBuffer* buffer, v2 pos, int size) {
    int xPos = (int)pos.x;
    int yPos = (int)pos.y;
    //x = bound(x,0,buffer->width);
    //y = bound(y,0,buffer->height);
    size /= 2;
    
    for (s32 y = max(0,yPos-size);
         y < min(yPos+size, (s32)buffer->height);
         y++) {
        u8* row = (u8*)buffer->bitmapMemory + buffer->pitch * y;
        for (s32 x = max(0, xPos-size);
             x < min(xPos+size, (s32)buffer->width);
             x++) {
            u8* pixel = row + buffer->bytesPerPixel * x;
            *((u32*)pixel) = 0x00FFFFFF;
        }
    }
}

internal void renderRectangle(GameOffscreenBuffer* buffer, PhysicsRect* r, u32 colour) {
    Polygon pRect;
    computeRectVertices(r, &pRect);
    v2* verts = pRect.verts;
    
    // Compute directions pointing inwards
    v2 dirs[4];
    for (int i=0;i<4;i++)
        dirs[i] = perp(verts[(i+1)%4] - verts[i]);

    int bottom = (int)buffer->height;
    int top = 0;
    int right = 0;
    int left = (int)buffer->width;
    for (int i=0;i<4;i++) {
        top = max(top,(int)verts[i].y);
        bottom = min(bottom,(int)verts[i].y);
        left = min(left,(int)verts[i].x);
        right = max(right,(int)verts[i].x);
    }
    top = bound(top,0,(int)buffer->height);
    bottom = bound(bottom,0,(int)buffer->height);
    left = bound(left,0,(int)buffer->width);
    right = bound(right,0,(int)buffer->width);
    
    for (int y = bottom; y < top; y++) {
        u8* row = (u8*)buffer->bitmapMemory + buffer->pitch * y;
        for (int x = left; x < right; x++) {
            u8* pixel = row + buffer->bytesPerPixel * x;
            v2 p = V2((f32)x,(f32)y);
            v2 topLeft = p - verts[1];
            v2 bottomRight = p - verts[3];
            if (dot(topLeft,dirs[0])>0 &&
                dot(topLeft,dirs[1])>0 &&
                dot(bottomRight,dirs[2])>0 &&
                dot(bottomRight,dirs[3])>0) {
            
                *((u32*)pixel) = colour;
            }
        }
    }
}

internal void setPixel(GameOffscreenBuffer* buffer, int x, int y, u32 colour) {
    if (x < 0 || y < 0 || x >= (int)buffer->width || y >= (int)buffer->height) return;

     u8* row = (u8*)buffer->bitmapMemory + buffer->pitch * y;  
     u8* pixel = row + buffer->bytesPerPixel * x;
     *((u32*)pixel) = colour;
}

//NOTE: This is using Bresenham's line alg.
internal void renderLine(GameOffscreenBuffer* buffer, v2 pos1, v2 pos2) {
    int x0=(int)pos1.x,x1=(int)pos2.x,y0=(int)pos1.y,y1=(int)pos2.y;
 
    int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
    int dy = abs(y1-y0), sy = y0<y1 ? 1 : -1; 
    int err = (dx>dy ? dx : -dy)/2, e2;
 
    for(;;){
        setPixel(buffer,x0,y0,0x00FFFFFF);
        if (x0==x1 && y0==y1) break;
        e2 = err;
        if (e2 >-dx) { err -= dy; x0 += sx; }
        if (e2 < dy) { err += dx; y0 += sy; }
    }
}
