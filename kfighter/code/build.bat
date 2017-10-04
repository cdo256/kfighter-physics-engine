REM @echo off

pushd "C:\Program Files (x86)\Microsoft Visual Studio\2017\Community\VC\Auxiliary\Build\"
call vcvarsall x86
popd

set compilerFlags=-MTd -Gm- -nologo -Oi -GR- -EHsc -EHa- -W3 -DKFIGHTER_SLOW=1 -DKFIGHTER_INTERNAL=1 -DKFIGHTER_WIN32=1 -FC -Z7
set linkerFlags=-incremental:no user32.lib gdi32.lib winmm.lib

IF NOT EXIST ..\build mkdir ..\build
pushd ..\build

REM "> NUL 2> NUL" hides all output including errors
del kfighter_*.pdb > NUL 2> NUL

REM 32-bit build
REM cl %compilerFlags% -Fmwin32_kfighter.map ..\code\win32_kfighter.cpp /link -subsystem:windows,5.1 %linkerFlags%

REM 64-bit build
cl %compilerFlags% -Fmkfighter.map ..\code\kfighter.cpp -LD /link -incremental:no -PDB:kfighter_%random%.pdb -EXPORT:gameUpdateAndRender
cl %compilerFlags% -Fmwin32_kfighter.map ..\code\win32_kfighter.cpp /link %linkerFlags%

popd
