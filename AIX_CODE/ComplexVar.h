// ComplexVar.h
// Marc Petit
// 04/07/15 
#pragma once
#include <math.h>

class ComplexVar 
{
	private:
		float d,q;
		
	public:
		
		inline ComplexVar ()
		{
			d = 0.f;
			q = 0.f;
		}	
		inline ComplexVar (float d_comp, float q_comp)
		{
			d = d_comp;
			q = q_comp;
		}
		inline ComplexVar operator+(const ComplexVar& dq)
		{
			ComplexVar dq_out;
			dq_out.d = this->d + dq.d;
			dq_out.q = this->q + dq.q;
			return dq_out;
		}
		inline ComplexVar operator-(const ComplexVar& dq)
		{
			ComplexVar dq_out;
			dq_out.d = this->d - dq.d;
			dq_out.q = this->q - dq.q;
			return dq_out;
		}
		inline ComplexVar operator-()
		{
			ComplexVar dq_out;
			dq_out.d = -this->d;
			dq_out.q = -this->q;
			return dq_out;
		}
		inline ComplexVar operator*(const ComplexVar& dq)
		{
			ComplexVar dq_out;
			dq_out.d = this->d * dq.d - this->q * dq.q;
			dq_out.q = this->d * dq.q + this->q * dq.d;
			return dq_out;
		}
		inline ComplexVar operator*(const float& scalar)
		{
			ComplexVar dq_out;
			dq_out.d = this->d * scalar;
			dq_out.q = this->q * scalar;
			return dq_out;
		}
		inline ComplexVar operator/(const float& scalar)
		{
			ComplexVar dq_out;
			dq_out.d = this->d / scalar;
			dq_out.q = this->q / scalar;
			return dq_out;
		}
		inline ComplexVar operator+=(const ComplexVar& dq)
		{
			this->d += dq.d;
			this->q += dq.q;
			return (*this);
		}
		inline ComplexVar scal_mul(const float& factor)
		{
			ComplexVar dq_out;
			dq_out.d = this->d * factor;
			dq_out.q = this->q * factor;
			return dq_out;
		}
		inline ComplexVar comp_mul(const ComplexVar& dq)
		{
			ComplexVar dq_out;
			dq_out.d = this->d * dq.d;
			dq_out.q = this->q * dq.q;
			return dq_out;
		}

		inline ComplexVar rotate(const float& theta)
		{
			ComplexVar dq_out;
			dq_out.d = this->d * cosf(theta) - this->q * sinf(theta);
			dq_out.q = this->d * sinf(theta) + this->q * cosf(theta);
			return dq_out;
		}

		inline static float real(const ComplexVar& dq)
		{
			return dq.d;
		}
		inline static float imag(const ComplexVar& dq)
		{
			return dq.q;
		}
		inline void clear()
		{
			this->d = 0;
			this->q = 0;
		}
		
		inline float get_d_comp() const 
		{
			return d;
		}
		inline float get_q_comp() const
		{
			return q;
		}
};


template <class T>
inline ComplexVar operator*(const T& factor, const ComplexVar& dq)
{
	return ComplexVar(dq.get_d_comp() * factor,dq.get_q_comp() * factor);
}
template <class T>
inline ComplexVar operator/(const T& factor, const ComplexVar& dq)
{
	return ComplexVar(factor/dq.get_d_comp(),factor/dq.get_q_comp());
}
inline ComplexVar expc(const ComplexVar& dq)
{	
	ComplexVar dq_out = ComplexVar(expf(dq.get_d_comp()),expf(dq.get_q_comp()));
	return dq_out;
}
