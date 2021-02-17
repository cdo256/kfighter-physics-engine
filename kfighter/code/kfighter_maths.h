#if !defined(KFIGHTER_MATHS_H)

#define KILOBYTES(x) ((x)*1024LL)
#define MEGABYTES(x) (KILOBYTES(x)*1024LL)
#define GIGABYTES(x) (MEGABYTES(x)*1024LL)

#define min(x,y) ((x)<(y)?(x):(y))
#define max(x,y) ((x)>(y)?(x):(y))
#define bound(x,lower,upper) (max((lower),min((upper),(x))))

#include <stdint.h>

#define rand __rand
#include <math.h>
#include <float.h>
#undef rand

typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;

typedef s8 b8;
typedef s16 b16;
typedef s32 b32;
typedef s64 b64;

typedef float f32;
typedef double f64;

global const f32 epsilon = 0.01f;
global const f32 pi = 3.14152635389793238f;

inline f32 sqr(f32 x);

struct v2 {
	f32 x, y;
};

inline v2 V2(f32 x, f32 y);

inline v2 operator+(v2 vec);
inline v2 operator-(v2 vec);
inline v2 operator+(v2 lhs, v2 rhs);
inline v2 operator-(v2 lhs, v2 rhs);
inline v2 operator*(f32 scalar, v2 vec);
inline v2 operator*(v2 vec, f32 scalar);
inline v2 operator/(v2 vec, f32 scalar);

inline v2 &operator+=(modified v2 &lhs, v2 rhs);
inline v2 &operator-=(modified v2 &lhs, v2 rhs);
inline v2 &operator*=(modified v2 &lhs, f32 scalar);
inline v2 &operator/=(modified v2 &lhs, f32 scalar);

inline bool operator==(v2 lhs, v2 rhs);
inline bool operator!=(v2 lhs, v2 rhs);

inline f32 sqrmag(v2 vec);
inline f32 mag(v2 vec);
inline v2 norm(v2 vec);
inline v2 perp(v2 vec); // = rotate(vec,pi/2)
inline f32 dot(v2 vec1, v2 vec2);
inline f32 cross(v2 vec1, v2 vec2);
inline v2 rotate(v2 vec, f32 angle);

//Fix angle between -pi and pi
inline f32 normAngle(f32 theta);

#define KFIGHTER_MATHS_H
#endif
