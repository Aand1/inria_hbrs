#ifndef _OBB2D_H
#define _OOB2D_H

#include <perception/Vector2.h>
#include <perception/dummy_perception.h>

class DummyPerception;

class OBB2D
{
public:
	OBB2D();
	~OBB2D();

	void computeBB(const Vector2& center, const double w, const double h, double angle)
	{
		Vector2 X( cos(angle), sin(angle));
		Vector2 Y(-sin(angle), cos(angle));

		X *= w / 2;
        Y *= h / 2;

        corner[0] = center - X - Y;
        corner[1] = center + X - Y;
        corner[2] = center + X + Y;
        corner[3] = center - X + Y;

	}

private:
	/** Corners of the box, where 0 is the lower left. */
    Vector2 corner[4];

    /** origin of the box */
    double origin[2];

};
#endif