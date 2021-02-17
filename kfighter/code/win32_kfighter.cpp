/* TODO: This is not a final platform layer
   - Save game location
   - Getting a handle to our own executable files
   - Asset loading
   - Threading
   - Raw input (multiple keyboards)
   - Sleep/timeBeginPeriod to save battery
   - Clip cursor
   - Fullscreen
   - Hide cursor
   - WMActivateApp (for when we're in the background)
   - Blitspeed improvement
   - Hardware acceleration (OpenGL)
   - Other keyboard layout support
*/

#include "kfighter.h"
#include "kfighter_util.h"

#include <windows.h>
#include <xinput.h>

global bool xinputLoaded = false;
#define XINPUT_GET_STATE(name) DWORD WINAPI name(DWORD dwUserIndex, XINPUT_STATE* pState)
#define XINPUT_SET_STATE(name) DWORD WINAPI name(DWORD dwUserIndex, XINPUT_VIBRATION* pVibration)

typedef XINPUT_GET_STATE(xinput_get_state);
typedef XINPUT_SET_STATE(xinput_set_state);

XINPUT_GET_STATE(xinputGetStateStub) {return 0;}
XINPUT_SET_STATE(xinputSetStateStub) {return 0;}

global xinput_get_state* XInputGetState_ = xinputGetStateStub;
global xinput_set_state* XInputSetState_ = xinputSetStateStub;
#define XInputGetState XInputGetState_
#define XInputSetState XInputSetState_

global u64 performanceFrequency;

//TODO: Does this need to be global?
//NOTE: Each global is initialised to zero by default
global bool globalRunning;
global bool globalPause;

global const int win32InputBufferMaxSize = 900; // 30 seconds at 30FPS

struct Win32State {
    u32 randomSeed;
    
    void* gameMemoryBlock;
    void* gameMemoryTempBlock;
    u64 gameMemorySize;
    
    bool recordingInput;
    bool playingInput;
    int inputRecordingPosition;
    int inputPlaybackPosition;
    int inputBufferSize;
    GameInput inputBuffer[win32InputBufferMaxSize];
};

internal void
win32ConcatStrings(
    in char* source1, in char* source2, out char* dest, int destCount) {
    int destPos = 0;
    for (int i = 0; source1[i]; i++) {
        assert(destPos < destCount);
        dest[destPos++] = source1[i];
    }
    for (int i = 0; source2[i]; i++) {
        assert(destPos < destCount);
        dest[destPos++] = source2[i];
    }
    assert(destPos < destCount);
    dest[destPos] = 0;
}

internal void win32LoadXInput() {
    HMODULE xinputLibrary = LoadLibrary("xinput1_3.dll");
    if (xinputLibrary) {
        xinputLoaded = true;
        XInputGetState = (xinput_get_state*)GetProcAddress(xinputLibrary, "XInputGetState");
        XInputSetState = (xinput_set_state*)GetProcAddress(xinputLibrary, "XInputSetState");
    }
}

struct Win32GameCode {
    HMODULE library;
    FILETIME lastDLLWriteTime;
    
    game_update_and_render* updateAndRender;

    b32 isValid;
};

internal FILETIME win32GetFileLastWriteTime(in char* name) {
    FILETIME result = {};

#if 0
    WIN32_FIND_DATAA findFileData;
    HANDLE findFileHandle = FindFirstFileA(name, &findFileData);
    
    if (findFileHandle != INVALID_HANDLE_VALUE) {
        FindClose(findFileHandle);
        result = findFileData.ftLastWriteTime;
    }
#else
    WIN32_FILE_ATTRIBUTE_DATA fileInfo;
    GetFileAttributesEx(name, GetFileExInfoStandard, &fileInfo);
    result = fileInfo.ftLastWriteTime;
#endif
    
    return result;
}

internal Win32GameCode
win32LoadGameCode(
    in char* executablePath, in char* sourceDLLName) {
    
    Win32GameCode result = {};

    char tempDLLName[MAX_PATH];

    win32ConcatStrings(executablePath, "kfighter_temp.dll", tempDLLName, sizeof(tempDLLName));

    result.lastDLLWriteTime = win32GetFileLastWriteTime(sourceDLLName);
    
    CopyFile(sourceDLLName, tempDLLName, FALSE);
    result.library = LoadLibrary(tempDLLName);
    if (result.library) {
        result.updateAndRender =
            (game_update_and_render*)GetProcAddress(result.library, "gameUpdateAndRender");
        result.isValid = !!result.updateAndRender;
    }

    if (!result.isValid) {
        result.updateAndRender = gameUpdateAndRenderStub;
    }

    return result;
}

internal void
win32UnloadGameCode(modified Win32GameCode* gameCode) {
    
    if (gameCode->library) {
        FreeLibrary(gameCode->library);
        gameCode->library = 0;
    }
    
    gameCode->isValid = false;
    gameCode->updateAndRender = gameUpdateAndRenderStub;
}

struct win32OffscreenBuffer {
    BITMAPINFO info;
    void* bitmapMemory;
    int width;
    int height;
    int pitch;
    int bytesPerPixel;
};

global win32OffscreenBuffer globalBackBuffer;

struct win32WindowDimension {
    int width;
    int height;
};

internal win32WindowDimension win32GetWindowDimension(in HWND hwnd) {
    win32WindowDimension ret;

    RECT rect;
    GetClientRect(hwnd, &rect);
    ret.width = rect.right - rect.left;
    ret.height = rect.bottom - rect.top;

    return ret;
}

internal void
win32ResizeDIBSection(
    modified win32OffscreenBuffer* buffer, int width, int height) {
    //TODO: Bulletproof this
    buffer->width = width;
    buffer->height = height;
    
    buffer->bytesPerPixel = 4;
    u16 bitsPerPixel = buffer->bytesPerPixel * 8;
    
    buffer->info.bmiHeader = {
        sizeof(BITMAPINFOHEADER),
        buffer->width,
        buffer->height,
        1,
        bitsPerPixel,
        BI_RGB
    };

    int bitmapMemorySize = (width*height)*buffer->bytesPerPixel;
    
    if (buffer->bitmapMemory) {
        //NOTE: If the size is not zero then this doesn't free anything
        VirtualFree(buffer->bitmapMemory, 0, MEM_RELEASE);
    }
    
    buffer->bitmapMemory = VirtualAlloc(0, bitmapMemorySize, MEM_COMMIT, PAGE_READWRITE);
    buffer->pitch = width*buffer->bytesPerPixel;
}


internal void
win32DisplayBufferInWindow(
    in win32OffscreenBuffer buffer,
    in HDC hdc,
    int windowWidth,
    int windowHeight,
    int x,
    int y,
    int width,
    int height) {

    //TODO: Correct aspect ratio
    StretchDIBits(hdc,
                  /*
                    x, y, width, height,
                    x, y, width, height,                  
                  */
                  0, 0, windowWidth, windowHeight,
                  0, 0, buffer.width, buffer.height,
                  buffer.bitmapMemory,
                  &buffer.info,
                  DIB_RGB_COLORS,
                  SRCCOPY);
}


internal void win32BeginRecordingInput(modified Win32State* state) {
    state->inputRecordingPosition = 0;
    state->inputBufferSize = 0;
    state->recordingInput = true;

    CopyMemory(
        state->gameMemoryTempBlock,
        state->gameMemoryBlock,
        (size_t)state->gameMemorySize);
}

internal void win32EndRecordingInput(modified Win32State* state) {
    state->recordingInput = false;
}

internal void win32BeginInputPlayback(modified Win32State* state) {
    state->inputPlaybackPosition = 0;
    state->playingInput = true;

    CopyMemory(
        state->gameMemoryBlock,
        state->gameMemoryTempBlock,
        (size_t)state->gameMemorySize);
}

internal void win32EndInputPlayback(modified Win32State* state) {
    state->playingInput = false;
}

internal void win32RecordInput(
    modified Win32State* state, in GameInput* input) {
    assert(state->recordingInput);
    state->inputBuffer[state->inputRecordingPosition++]
        = *input;
    if (state->inputRecordingPosition > state->inputBufferSize) {
        state->inputBufferSize = state->inputRecordingPosition;
    }
    assert(state->inputBufferSize <= win32InputBufferMaxSize);
}

internal void win32PlaybackInput(
    modified Win32State* state, out GameInput* input) {
    assert(state->playingInput);
    *input = state->inputBuffer[state->inputPlaybackPosition++];
    if (state->inputPlaybackPosition >= state->inputBufferSize) {
        win32EndInputPlayback(state);
        win32BeginInputPlayback(state);
    }
}

LRESULT CALLBACK
win32MainWindowCallback(
    HWND   hwnd,
    UINT   uMsg,
    WPARAM wParam,
    LPARAM lParam) {

    LRESULT lResult = 0;
    
    switch (uMsg) {
        case WM_SIZE: {
            OutputDebugString("WM_SIZE\n");

            win32WindowDimension dimension = win32GetWindowDimension(hwnd);
            win32ResizeDIBSection(&globalBackBuffer, dimension.width, dimension.height);            
        } break;

        case WM_PAINT: {
            PAINTSTRUCT paintStruct;
            HDC deviceContext = BeginPaint(hwnd, &paintStruct);
            
            win32WindowDimension dimension = win32GetWindowDimension(hwnd);
            
            int x = paintStruct.rcPaint.left;
            int y = paintStruct.rcPaint.top;
            int width = paintStruct.rcPaint.right-paintStruct.rcPaint.left;
            int height = paintStruct.rcPaint.bottom-paintStruct.rcPaint.top;
            
            win32DisplayBufferInWindow(globalBackBuffer, deviceContext, dimension.width, dimension.height, x, y, width, height);
            EndPaint(hwnd, &paintStruct);
        } break;
            
        case WM_DESTROY: {
            //TODO: Handle as error?
            OutputDebugString("WM_DESTROY\n");
            globalRunning = false;
        } break;

        case WM_CLOSE: {
            //TODO: Message to the user?
            OutputDebugString("WM_CLOSE\n");
            globalRunning = false;
        } break;

        case WM_ACTIVATEAPP: {
            OutputDebugString("WM_ACTIVATEAPP\n");
        } break;

        case WM_SYSKEYDOWN:
        case WM_SYSKEYUP:
        case WM_KEYDOWN: 
        case WM_KEYUP: {
            assert(!"Keyboard input came in through non-dispatch message");
        } break;
                                   
        default: {
            lResult = DefWindowProc(hwnd, uMsg, wParam, lParam);
        } break;
    }

    return lResult;
}

internal void
win32ProcessKeyboardMessage(
    out GameButtonState* state, b32 isDown) {
    //TODO: This keeps triggering when exiting the window
    //assert(state->endedDown != isDown);
    state->endedDown = isDown;
    state->halfTransitionCount++;
}

void win32ProcessPendingMessages(
    modified Win32State* state,
    modified GameControllerInput* keyboardController) {
    
    MSG msg;
    while (PeekMessage(&msg, 0, 0, 0, PM_REMOVE)) {
        switch (msg.message) {
            case WM_QUIT: {
                globalRunning = false;
            } break;
                
            case WM_SYSKEYDOWN:
            case WM_SYSKEYUP:
            case WM_KEYDOWN: 
            case WM_KEYUP: {
                u32 vkCode = (u32)msg.wParam;
                b32 wasDown = ((msg.lParam & (1<<30)) != 0);
                b32 isDown = ((msg.lParam & (1<<31)) == 0);
                if (wasDown != isDown) {
                    switch (vkCode) {
                        case 'W': {} break;
                        case 'A': {} break;
                        case 'S': {} break;
                        case 'D': {} break;
                        case 'Q': {} break;
                        case 'E': {} break;
                        case VK_UP: {
                            win32ProcessKeyboardMessage(&keyboardController->up, isDown);
                        } break;
                        case VK_DOWN: {
                            win32ProcessKeyboardMessage(&keyboardController->down, isDown);
                        } break;
                        case VK_LEFT: {
                            win32ProcessKeyboardMessage(&keyboardController->left, isDown);
                        } break;
                        case VK_RIGHT: {
                            win32ProcessKeyboardMessage(&keyboardController->right, isDown);
                        } break;
                        case VK_ESCAPE: {
                            if (isDown) globalRunning = false;
                        } break;
                        case VK_SPACE: {
                            win32ProcessKeyboardMessage(&keyboardController->aButton, isDown);
                        } break;
#if KFIGHTER_INTERNAL
                        case 'P': {
                            if (wasDown) {
                                globalPause = !globalPause;
                            }
                        } break;
                        case 'L': {
                            if (wasDown) {
                                assert(!state->recordingInput || !state->playingInput);
                                if (state->recordingInput) {
                                    win32EndRecordingInput(state);
                                    win32BeginInputPlayback(state);
                                    OutputDebugStringA("Ending input recording\n");
                                    OutputDebugStringA("Beginning input playback\n");
                                } else if (state->playingInput) {
                                    win32EndInputPlayback(state);
                                    OutputDebugStringA("Ending input playback\n");
                                } else {
                                    win32BeginRecordingInput(state);
                                    OutputDebugStringA("Beginning input recording\n");
                                }
                            }
                        } break;
                        case 'R': {
                            if (wasDown) {
                                char buffer[256];
                                wsprintf(buffer, "Seed: %d\n",state->randomSeed);
                                OutputDebugStringA(buffer);
#if 0
                                //TODO: Wy doesn't this work
                                ZeroMemory(
                                    state->gameMemoryBlock,
                                    state->gameMemorySize);
#else
                                for (u64 i = 0; i < state->gameMemorySize/8; i++)
                                    ((u64*)state->gameMemoryBlock)[i] = 0;
#endif
                            }
                        } break;
                        case VK_OEM_4: { // "["
                            if (wasDown) {
                                state->randomSeed--;
                                char buffer[256];
                                wsprintf(buffer, "Seed: %d\n",state->randomSeed);
                                OutputDebugStringA(buffer);

                                for (u64 i = 0; i < state->gameMemorySize/8; i++)
                                    ((u64*)state->gameMemoryBlock)[i] = 0;
                            }
                        } break;
                        case VK_OEM_6: { // "]"
                            if (wasDown) {
                                state->randomSeed++;
                                char buffer[256];
                                wsprintf(buffer, "Seed: %d\n",state->randomSeed);
                                OutputDebugStringA(buffer);
                                
                                for (u64 i = 0; i < state->gameMemorySize/8; i++)
                                    ((u64*)state->gameMemoryBlock)[i] = 0;
                            } 
                        } break;
#endif
                    }
                }
            } break;
                            
            default: {
                TranslateMessage(&msg);
                DispatchMessage(&msg);
            } break;
        }
    }    
}

inline u64 win32GetTicks() {
    LARGE_INTEGER result;
    QueryPerformanceCounter(&result);
    return result.QuadPart;
}

inline f32 win32GetSecondsElapsedSince(u64 start) {
    u64 end = win32GetTicks();
    u64 countsElapsed = end - start;
    f32 secondsElapsed = (f32)countsElapsed / (f32)performanceFrequency;
    return secondsElapsed;
}

internal void win32ProcessXInputDigitalButton(
    DWORD xinputButtonState,
    in GameButtonState* oldState,
    DWORD buttonBit,
    out GameButtonState* newState) {

    newState->endedDown = ((xinputButtonState & buttonBit) == buttonBit);
    newState->halfTransitionCount =
        (oldState->endedDown != newState->endedDown) ? 1 : 0;
}

int CALLBACK WinMain(
    HINSTANCE hInstance,
    HINSTANCE hPrevInstance,
    in LPSTR  lpCmdLine,
    int       nCmdShow) {

    //NOTE: Never use MAX_PATH in user facing code
    char executableFilepath[MAX_PATH];
    DWORD sizeOfFileName = GetModuleFileNameA(0, executableFilepath, sizeof(executableFilepath));
    char* endOfExecutableFilepath = 0;
    for (char* scan = executableFilepath; *scan; scan++) {
        if (*scan == '\\') {
            endOfExecutableFilepath = scan+1;
        }
    }
    *endOfExecutableFilepath = 0;
    char sourceDLLName[MAX_PATH];        
    win32ConcatStrings(executableFilepath, "kfighter.dll", sourceDLLName, sizeof(sourceDLLName));

    globalPause = false;
    
#if KFIGHTER_INTERNAL
    LPVOID baseAddress = (LPVOID)GIGABYTES(2048);
#else
    LPVOID baseAddress = 0;
#endif

    //TODO: Does putting this struct on the heap make things slower?
    Win32State* state = (Win32State*)VirtualAlloc(
        0,
        sizeof(Win32State),
        MEM_COMMIT,
        PAGE_READWRITE);
    //TODO: What if this fails?
    
    state->randomSeed = initialSeed;

    GameMemory memory = {};
    memory.permanentStorageSize = MEGABYTES(4);
    memory.transientStorageSize = MEGABYTES(16);

    state->gameMemorySize = memory.permanentStorageSize
        + memory.transientStorageSize;
    state->gameMemoryBlock = VirtualAlloc(
        baseAddress,
        (size_t)state->gameMemorySize*2,
        MEM_COMMIT | MEM_RESERVE,
        PAGE_READWRITE);
    state->gameMemoryTempBlock = (u8*)state->gameMemoryBlock
        + state->gameMemorySize;
            
    //NOTE: This is cleared to 0
    memory.permanentStorage = state->gameMemoryBlock;
    memory.transientStorage = (u8*)memory.permanentStorage
        + memory.permanentStorageSize;

    assert(memory.permanentStorage && memory.transientStorage);
    
    win32ResizeDIBSection(&globalBackBuffer, 800, 600);

    win32LoadXInput();
    Win32GameCode gameCode = win32LoadGameCode(executableFilepath, sourceDLLName);
    
    WNDCLASSA windowClass = {};
    
    windowClass.style = CS_HREDRAW|CS_VREDRAW|CS_OWNDC;
    windowClass.lpfnWndProc = &win32MainWindowCallback;
    windowClass.hInstance = hInstance;
    windowClass.lpszClassName = "KFighterWindowClass";

    LARGE_INTEGER performanceFrequencyResult;
    QueryPerformanceFrequency(&performanceFrequencyResult);
    performanceFrequency = performanceFrequencyResult.QuadPart;

    //NOTE: Set up the scheduler to check every millisecond for sleeping
    //      This allows us to better maintain a constant FPS
    UINT desiredSchedulerGranularityMilliseconds = 1;
    b32 isSleepGranular = (timeBeginPeriod(desiredSchedulerGranularityMilliseconds) == TIMERR_NOERROR);

    //TODO: Get this from the monitor so we are vsynced
    u32 monitorRefreshHz = 60;
    u32 gameUpdateHz = 30;
    f32 targetSecondsPerFrame = 1.f / (f32)gameUpdateHz;
    
    if (RegisterClass(&windowClass)) {
        HWND hwnd = CreateWindowEx(
            0,
            windowClass.lpszClassName,
            "K-Fighter",
            WS_OVERLAPPEDWINDOW|WS_VISIBLE,
            CW_USEDEFAULT,
            CW_USEDEFAULT,
            CW_USEDEFAULT,
            CW_USEDEFAULT,
            0,
            0, //TODO: Might need HMENU in the future
            hInstance,
            0);
        
        if (hwnd) {
            GameInput inputArray[3] = {};
            GameInput* newInput = &inputArray[0];
            GameInput* oldInput = &inputArray[1];
            GameInput* playbackInput = &inputArray[2];
            
            u64 lastPerformaceCounter = win32GetTicks();
            f32 secondsElapsedForLastFrame = 0.f;
            
            u64 frameCount = 0;

            globalRunning = true;
            while (globalRunning) {
                FILETIME curDLLWriteTime = win32GetFileLastWriteTime(sourceDLLName);
                if (curDLLWriteTime.dwLowDateTime != gameCode.lastDLLWriteTime.dwLowDateTime
                    || curDLLWriteTime.dwHighDateTime != gameCode.lastDLLWriteTime.dwHighDateTime) {
                    win32UnloadGameCode(&gameCode);
                    gameCode = win32LoadGameCode(executableFilepath, sourceDLLName);
                }
                
                GameControllerInput* keyboardController = &newInput->controllers[0];
                //TODO: Zeroing macro
                GameControllerInput zeroController = {};
                *keyboardController = zeroController;
                
                for (u32 i = 0; i < arrayCount(newInput->controllers); i++) {
                    GameControllerInput* oldController = &oldInput->controllers[i];
                    GameControllerInput* newController = &newInput->controllers[i];
                    for (u32 j = 0; j < arrayCount(newController->buttons); j++) {
                        GameButtonState* oldState = &oldController->buttons[j];
                        GameButtonState* newState = &newController->buttons[j];
                        newState->endedDown = oldState->endedDown;
                    }
                }

                win32ProcessPendingMessages(state, keyboardController);
                
                if (xinputLoaded) {
                    //TODO: Should we poll this more frequently?
                    u32 maxControllerCount = XUSER_MAX_COUNT;
                    if (maxControllerCount > arrayCount(newInput->controllers))
                        maxControllerCount = arrayCount(newInput->controllers);
                    
                    for (u32 controllerIndex = 0; controllerIndex < maxControllerCount; controllerIndex++) {
                        u32 ourControllerIndex = controllerIndex + 1; // Since keyboard is always controller1 in our system
                        GameControllerInput* oldController = &oldInput->controllers[ourControllerIndex];
                        GameControllerInput* newController = &newInput->controllers[ourControllerIndex];

                        XINPUT_STATE controllerState;
                        if (XInputGetState(controllerIndex,&controllerState) == ERROR_SUCCESS) {
                            //TODO: See if the dwPacketNumber increments too fast
                            XINPUT_GAMEPAD* gamepad = &controllerState.Gamepad;

                            b32 start = (b32)(gamepad->wButtons & XINPUT_GAMEPAD_START);
                            b32 back = (b32)(gamepad->wButtons & XINPUT_GAMEPAD_BACK);
                            b32 leftShoulder = (b32)(gamepad->wButtons & XINPUT_GAMEPAD_LEFT_SHOULDER);
                            b32 rightShoulder = (b32)(gamepad->wButtons & XINPUT_GAMEPAD_RIGHT_SHOULDER);
                            
                            s16 lStickX = gamepad->sThumbLX;
                            s16 lStickY = gamepad->sThumbLY;
                            s16 rStickX = gamepad->sThumbRX;
                            s16 rStickY = gamepad->sThumbRY;

                            newController->isAnalog = true;
                            newController->lStick.startX = oldController->lStick.endX;
                            newController->lStick.startY = oldController->lStick.endY;
                            newController->rStick.startX = oldController->rStick.endX;
                            newController->rStick.startY = oldController->rStick.endY;

                            newController->lStick.endX = (f32)lStickX / 32768.f;
                            newController->lStick.endY = (f32)lStickY / 32768.f;
                            newController->rStick.endX = (f32)rStickX / 32768.f;
                            newController->rStick.endY = (f32)rStickY / 32768.f;
                            newController->lStick.maxX = newController->lStick.minX
                                = newController->lStick.endX;
                            newController->lStick.maxY = newController->lStick.minY
                                = newController->lStick.endY;
                            newController->rStick.maxX = newController->rStick.minX
                                = newController->rStick.endX;
                            newController->rStick.maxY = newController->rStick.minY
                                = newController->rStick.endY;

                            win32ProcessXInputDigitalButton(
                                gamepad->wButtons,
                                &oldController->up,
                                XINPUT_GAMEPAD_DPAD_UP,
                                &newController->up);
                            win32ProcessXInputDigitalButton(
                                gamepad->wButtons,
                                &oldController->down,
                                XINPUT_GAMEPAD_DPAD_DOWN,
                                &newController->down);                            
                            win32ProcessXInputDigitalButton(
                                gamepad->wButtons,
                                &oldController->left,
                                XINPUT_GAMEPAD_DPAD_LEFT,
                                &newController->left);
                            win32ProcessXInputDigitalButton(
                                gamepad->wButtons,
                                &oldController->right,
                                XINPUT_GAMEPAD_DPAD_RIGHT,
                                &newController->right);

                            win32ProcessXInputDigitalButton(
                                gamepad->wButtons,
                                &oldController->aButton,
                                XINPUT_GAMEPAD_A,
                                &newController->aButton);
                            win32ProcessXInputDigitalButton(
                                gamepad->wButtons,
                                &oldController->bButton,
                                XINPUT_GAMEPAD_B,
                                &newController->bButton);                            
                            win32ProcessXInputDigitalButton(
                                gamepad->wButtons,
                                &oldController->xButton,
                                XINPUT_GAMEPAD_X,
                                &newController->xButton);
                            win32ProcessXInputDigitalButton(
                                gamepad->wButtons,
                                &oldController->yButton,
                                XINPUT_GAMEPAD_Y,
                                &newController->yButton);

                            win32ProcessXInputDigitalButton(
                                gamepad->wButtons,
                                &oldController->start,
                                XINPUT_GAMEPAD_START,
                                &newController->start);
                            win32ProcessXInputDigitalButton(
                                gamepad->wButtons,
                                &oldController->back,
                                XINPUT_GAMEPAD_BACK,
                                &newController->back);
                            win32ProcessXInputDigitalButton(
                                gamepad->wButtons,
                                &oldController->lShoulder,
                                XINPUT_GAMEPAD_LEFT_SHOULDER,
                                &newController->lShoulder);
                            win32ProcessXInputDigitalButton(
                                gamepad->wButtons,
                                &oldController->rShoulder,
                                XINPUT_GAMEPAD_RIGHT_SHOULDER,
                                &newController->rShoulder);
                            
                            u8 leftTrigger = gamepad->bLeftTrigger;
                            u8 rightTrigger = gamepad->bRightTrigger;

                            //newController->up = dpadUp;
                        } else {
                            //NOTE: This controller is unavailable
                        }
                    }
                }

                win32WindowDimension dimension = win32GetWindowDimension(hwnd);
                GameOffscreenBuffer buffer = {};
                buffer.bitmapMemory = globalBackBuffer.bitmapMemory;
                buffer.width = globalBackBuffer.width;
                buffer.height = globalBackBuffer.height;
                buffer.pitch = globalBackBuffer.pitch;
                buffer.bytesPerPixel = globalBackBuffer.bytesPerPixel;                

                GameInput* input = newInput;
                if (state->recordingInput) {
                    win32RecordInput(state, newInput);
                }
                if (state->playingInput) {
                    win32PlaybackInput(state, playbackInput);
                    input = playbackInput;
                }
                f32 dt = secondsElapsedForLastFrame;
                //TODO: should this go inside the game code?
                if (dt > 0.1f) dt = 0.1f; // cap time jumps to prevent buggy physics
                gameCode.updateAndRender(
                    dt,
                    state->randomSeed,
                    &memory,
                    input,
                    &buffer);
                
                HDC deviceContext = GetDC(hwnd);            
                win32DisplayBufferInWindow(globalBackBuffer, deviceContext, dimension.width, dimension.height, 0, 0, dimension.width, dimension.height);           

                GameInput* temp = newInput;
                newInput = oldInput;
                oldInput = temp;
                
                f32 secondsElapsedForFrame = win32GetSecondsElapsedSince(lastPerformaceCounter);
#if 1
                if (secondsElapsedForFrame < targetSecondsPerFrame) {
                    while (secondsElapsedForFrame < targetSecondsPerFrame) {
                        if (isSleepGranular) {
                            u32 sleepMilliseconds = (u32)((targetSecondsPerFrame - secondsElapsedForFrame)*1000);
                            Sleep(sleepMilliseconds);
                        }
                        secondsElapsedForFrame = win32GetSecondsElapsedSince(lastPerformaceCounter);
                    }
                } else {
                    //TODO: Log missed frames
                }
#endif
                secondsElapsedForLastFrame = secondsElapsedForFrame;
                
                s32 microSecondsElapsed = (s32)(secondsElapsedForFrame*1000000);
                s32 fps = 1000000 / microSecondsElapsed;

                if (frameCount % 31 == 0) {
                    char performancePrintBuffer[256];
                    wsprintf(performancePrintBuffer, "%d microseconds per frame, %dFPS\n", microSecondsElapsed, fps);
                    OutputDebugStringA(performancePrintBuffer);
                }
                lastPerformaceCounter = win32GetTicks();
                    
                frameCount++;
            }
        } else {
            //TODO: Log error
        }
    } else {
        //TODO: Log error
    }
    
    return 0;
}
