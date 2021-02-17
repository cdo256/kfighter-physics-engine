inline f32 sqr(f32 x) {return x*x;}
inline v2 V2(f32 x, f32 y) {
	v2 result;
	result.x = x;
	result.y = y;
	return result;
}

inline v2 operator+(v2 vec) {return vec;}
inline v2 operator-(v2 vec) {
	v2 result;
	result.x = -vec.x;
	result.y = -vec.y;
	return result;
}
inline v2 operator+(v2 lhs, v2 rhs) {
	v2 result;
	result.x = lhs.x + rhs.x;
	result.y = lhs.y + rhs.y;
	return result;
}
inline v2 operator-(v2 lhs, v2 rhs) {return lhs+(-rhs);}
inline v2 operator*(f32 scalar, v2 vec) {
	v2 result;
	result.x = scalar * vec.x;
	result.y = scalar * vec.y;
	return result;
}
inline v2 operator*(v2 vec, f32 scalar) {return scalar*vec;}
inline v2 operator/(v2 vec, f32 scalar) {
	if (scalar == 0) return vec;
	else return (1/scalar)*vec;
}

inline v2 &operator+=(modified v2 &lhs, v2 rhs) {return lhs = lhs+rhs;}
inline v2 &operator-=(modified v2 &lhs, v2 rhs) {return lhs = lhs-rhs;}
inline v2 &operator*=(modified v2 &lhs, f32 scalar) {return lhs = lhs*scalar;}
inline v2 &operator/=(modified v2 &lhs, f32 scalar) {
	if (scalar == 0) return lhs;
	else return lhs = lhs/scalar;
}

inline bool operator==(v2 lhs, v2 rhs) {return mag(lhs-rhs) < epsilon;}
inline bool operator!=(v2 lhs, v2 rhs) {return !(lhs == rhs);}

inline f32 sqrmag(v2 vec) {return sqr(vec.x) + sqr(vec.y);}
inline f32 mag(v2 vec) {return sqrtf(sqrmag(vec));}
inline v2 norm(v2 vec) {
	f32 magnitude = mag(vec);
	if (magnitude == 0)
	    return V2(1,0);
	return vec/mag(vec);
}
inline v2 perp(v2 vec) {return V2(-vec.y,vec.x);}
inline f32 dot(v2 vec1, v2 vec2) {
	f32 p1 = vec1.x*vec2.x;
	f32 p2 = vec1.y*vec2.y;
	f32 result = p1+p2;
	return result;
}
inline f32 cross(v2 vec1, v2 vec2) {
	return vec1.x*vec2.y - vec1.y*vec2.x;
}
inline v2 rotate(v2 vec, f32 angle) {
	v2 result;
	result.x = cosf(angle)*vec.x - sinf(angle)*vec.y;
	result.y = sinf(angle)*vec.x + cosf(angle)*vec.y;
	return result;
}
inline f32 normAngle(f32 theta) {
	if (theta < -pi) theta = pi-fmodf(-theta-pi, 2.f*pi);
	else theta = fmodf(theta+pi, 2.f*pi)-pi;
	return theta;
}
