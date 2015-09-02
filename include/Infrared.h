#pragma once
#ifndef INFRARED_H
#define INFRARED_H

#include <gmapping\utils\GPoint.h>

struct Infrared 
{
public:
	Infrared() {};
	Infrared(double range, GMapping::OrientedPoint pose, double max_range);

	GMapping::OrientedPoint m_pose;
	double m_Range;
	double m_MaxRange;
	//double s, c;
};

inline Infrared::Infrared(double range, GMapping::OrientedPoint pose, double max_range) 
{
	m_Range = range;
	m_pose = pose;
	m_MaxRange = max_range;
}

#endif INFRARED_H