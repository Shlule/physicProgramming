#pragma once
#include "Body.h"
#include "Shape.h"
#include"Contact.h"

class Intersection
{
public: 
	static bool Intersect(Body& a, Body& b, Contact& contact);
};

