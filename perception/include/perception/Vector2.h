/*References: http://codereview.stackexchange.com/questions/5856/mathematical-vector2-class-implementation
              http://leetnightshade.com/vector2-class*/

//VECTOR2 H
#ifndef VECTOR2_H
#define VECTOR2_H

//INCLUDES
#include <math.h>

//DEFINE TYPES
typedef float float64;

//VECTOR2 CLASS
class Vector2
{
public:
	//MEMBERS
	float64 x;
	float64 y;

	//CONSTRUCTORS
	Vector2(void) : x(0), y(0) { }
	Vector2(float64 xValue, float64 yValue) : x(xValue), y(yValue) { }
	Vector2(const Vector2 & v) : x(v.x), y(v.y) { }
	//Vector2(const Vector2 * v) : x(v->x), y(v->y) { }

	//DECONSTRUCTOR
	~Vector2() { }

	//VECTOR2 TO VECTOR2 OPERATIONS
	inline const Vector2 operator + (const Vector2 & v) const { return Vector2(x + v.x, y + v.y); }
	inline const Vector2 operator - (const Vector2 & v) const { return Vector2(x - v.x, y - v.y); }

    //VECTOR2 TO THIS OPERATIONS
	void operator*=(const float& num)
	{ 
		x *= num;
		y *= num;
	}

};

#endif
//ENDFILE
