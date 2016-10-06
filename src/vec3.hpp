#ifndef __VEC3_HPP__
#define __VEC3_HPP__


class vec3
{
public:
	float x;
	float y;
	float z;
	vec3(const float x, const float y, const float z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}
	vec3()
	{
		x = y = z = 0.0;
	}
	vec3 operator+(const vec3 &q) const
	{
		return vec3(x+q.x, y+q.y, z+q.z);
	}
	vec3 operator-(const vec3 &q) const
	{
		return vec3(x-q.x, y-q.y, z-q.z);
	}
	vec3 operator*(const float &s) const
	{
		return vec3(x*s, y*s, z*s);
	}
	vec3 operator/(const float &s) const
	{
		return vec3(x/s, y/s, z/s);
	}
	vec3 &operator+=(const vec3 &q)
	{
		*this = *this + q;
		return *this;
	}
	float dot(const vec3 &q) const
	{
		return x*q.x + y*q.y + z*q.z;
	}
	float norm() const
	{
		return sqrtf(dot(*this));
	}
};

#endif

