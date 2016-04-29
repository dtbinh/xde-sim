#ifndef COMMON_H
#define COMMON_H
#pragma once

#define _USE_MATH_DEFINES

#include <math.h>
#include <exception>
#include <iostream>

#if defined(_WIN32)
#include <Windows.h>

#elif defined(__unix__) || defined(__unix) || defined(unix) || (defined(__APPLE__) && defined(__MACH__))
#include <unistd.h>    /* POSIX flags */
#include <time.h>    /* clock_gettime(), time() */
#include <sys/time.h>    /* gethrtime(), gettimeofday() */
#include <X11/Xlib.h>

#if defined(__MACH__) && defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_time.h>
#endif

#else
#error "Unable to define Common::getRealTime( ) for an unknown OS."
#endif

namespace Common
{

	class MyException: public std::exception
	{
	private:
		std::string _msg;

	public:
		MyException(std::string msg): std::exception() { _msg = msg; };
		virtual const char* what() const throw()
		{
			return _msg.c_str();
		}
	};

	/*!
	* \brief
	* Returns the Fast Inverse Square Root.
	*/
	template<typename R>
	R finvsqrt( R number )
	{
		float num = static_cast<float>(number);
		long i;
		float x2, y;
		const float threehalfs = 1.5F;

		x2 = number * 0.5F;
		y  = number;
		i  = * ( long * ) &y;                       // evil floating point bit level hacking
		i  = 0x5f375a86 - ( i >> 1 );               // Chris Lomont's constant
		y  = * ( float * ) &i;
		y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration
		//      y  = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed

		return static_cast<R>y;
	};

	/*
	* Author:  David Robert Nadeau
	* Modified by: José Magno Mendes Filho
	* Site:    http://NadeauSoftware.com/
	* License: Creative Commons Attribution 3.0 Unported License
	*          http://creativecommons.org/licenses/by/3.0/deed.en_US
	*/

	/*!
	* \brief
	* Returns the real time, in seconds, or -1.0 if an error occurred.
	*
	* Time is measured since an arbitrary and OS-dependent start time.
	* The returned real time is only useful for computing an elapsed time
	* between two calls to this function.
	*/
	template<typename R>
	R getRealTime()
	{
		#if defined(_WIN32)
			FILETIME tm;
			ULONGLONG t;
			#if defined(NTDDI_WIN8) && NTDDI_VERSION >= NTDDI_WIN8
				/* Windows 8, Windows Server 2012 and later. ---------------- */
				GetSystemTimePreciseAsFileTime( &tm );
			#else
				/* Windows 2000 and later. ---------------------------------- */
				GetSystemTimeAsFileTime( &tm );
			#endif
			t = ((ULONGLONG)tm.dwHighDateTime << 32) | (ULONGLONG)tm.dwLowDateTime;
			return static_cast<R>((double)t / 10000000.0);

		#elif (defined(__hpux) || defined(hpux)) || ((defined(__sun__) || defined(__sun) || defined(sun)) && (defined(__SVR4) || defined(__svr4__)))
			/* HP-UX, Solaris. ------------------------------------------ */
			return static_cast<R>((double)gethrtime( ) / 1000000000.0);

		#elif defined(__MACH__) && defined(__APPLE__)
			/* OSX. ----------------------------------------------------- */
			static double timeConvert = 0.0;
			if ( timeConvert == 0.0 )
			{
				mach_timebase_info_data_t timeBase;
				(void)mach_timebase_info( &timeBase );
				timeConvert = (double)timeBase.numer /
				(double)timeBase.denom /
				1000000000.0;
			}
			return static_cast<R>((double)mach_absolute_time( ) * timeConvert);

		#elif defined(_POSIX_VERSION)
			/* POSIX. --------------------------------------------------- */
			#if defined(_POSIX_TIMERS) && (_POSIX_TIMERS > 0)
			{
				struct timespec ts;
				#if defined(CLOCK_MONOTONIC_PRECISE)
					/* BSD. --------------------------------------------- */
					const clockid_t id = CLOCK_MONOTONIC_PRECISE;
				#elif defined(CLOCK_MONOTONIC_RAW)
					/* Linux. ------------------------------------------- */
					const clockid_t id = CLOCK_MONOTONIC_RAW;
				#elif defined(CLOCK_HIGHRES)
					/* Solaris. ----------------------------------------- */
					const clockid_t id = CLOCK_HIGHRES;
				#elif defined(CLOCK_MONOTONIC)
					/* AIX, BSD, Linux, POSIX, Solaris. ----------------- */
					const clockid_t id = CLOCK_MONOTONIC;
				#elif defined(CLOCK_REALTIME)
					/* AIX, BSD, HP-UX, Linux, POSIX. ------------------- */
					const clockid_t id = CLOCK_REALTIME;
				#else
					const clockid_t id = (clockid_t)-1;    /* Unknown. */
				#endif /* CLOCK_* */
				if ( id != (clockid_t)-1 && clock_gettime( id, &ts ) != -1 )
					return static_cast<R>((double)ts.tv_sec +
					(double)ts.tv_nsec / 1000000000.0);
					/* Fall thru. */
			}
			#endif /* _POSIX_TIMERS */

			/* AIX, BSD, Cygwin, HP-UX, Linux, OSX, POSIX, Solaris. ----- */
			struct timeval tm;
			gettimeofday( &tm, NULL );
			return static_cast<R>((double)tm.tv_sec + (double)tm.tv_usec / 1000000.0);
		#else
			return static_cast<R>(-1.0);        /* Failed. */
		#endif
	};

	/* Map angles (:math:`\\theta \in R`) to signed angles
	* (:math:`\\theta \in [-pi, +pi)`).
	*/
	template <typename R>
	R wrapToPi(R angle)
	{
		while (angle < -M_PI)
			angle += 2*M_PI;
		while (angle >= M_PI)
			angle -= 2*M_PI;
		return angle;
	};

	/* Map angles (:math:`\\theta \in R`) to unsigned angles
	* (:math:`\\theta \in [0, 2\pi)`).
	*/
	template <typename R>
	R wrapTo2Pi(R angle)
	{
		while (angle < 0.0)
			angle += 2*M_PI;
		while (angle >= 2*M_PI)
			angle -= 2*M_PI;
		return angle;
	};

	// Unused SFINAE example
	template <typename T>
	class has_size {
	private:
		typedef char Yes;
		typedef Yes No[2];

		template <typename U, U> struct really_has;

		template<typename C> static Yes& Test(really_has <size_t (C::*)() const, &C::size>*);
		template<typename C> static Yes& Test(really_has <size_t (C::*)(), &C::size>*);

		template<typename> static No& Test(...);
	 
	public:
	    static bool const value = sizeof(Test<T>(0)) == sizeof(Yes);
	};

	namespace has_insertion_operator_impl
	{
		typedef char no;
		typedef char yes[2];

		struct any_t
		{
			template<typename T> any_t( T const& );
		};

		no operator<<( std::ostream const&, any_t const& );

		yes& test( std::ostream& );
		no test( no );

		template<typename T>
		struct has_insertion_operator
		{
			static std::ostream &s;
			static T const &t;
			static bool const value = sizeof( test(s << t) ) == sizeof( yes );
		};
	}

	template<typename T>
	struct has_insertion_operator :
	  has_insertion_operator_impl::has_insertion_operator<T> {};


	// A container is not printable but the return of the call has to be
	template <typename T, typename A, template<typename, typename> class C>
	char safe_print(C<T, A>) {return '\0';};

	template <typename E, typename T, typename A, template<typename, typename, typename> class S>
	std::string safe_print(S<E,T,A> toPrint) {return toPrint;}; 

	template <typename>
	short safe_print(short toPrint) {return toPrint;}; 

	template <typename>
	unsigned short safe_print(unsigned short toPrint) {return toPrint;}; 

	template <typename>
	int safe_print(int toPrint) {return toPrint;}; 

	template <typename>
	unsigned int safe_print(unsigned int toPrint) {return toPrint;}; 

	template <typename>
	long safe_print(long toPrint) {return toPrint;}; 

	template <typename>
	unsigned long safe_print(unsigned long toPrint) {return toPrint;}; 

	template <typename>
	float safe_print(float toPrint) {return toPrint;}; 

	template <typename>
	double safe_print(double toPrint) {return toPrint;}; 

	template <typename>
	long double safe_print(long double toPrint) {return toPrint;};

	template <typename>
	bool safe_print(bool toPrint) {return toPrint;};

	// For all we know, T could be a container thus the safe_print overloads
	template <typename T, typename A, template<typename, typename> class C>
	void printContainer(C<T, A> toIterate)
	{
		std::cout << "[";
		for (C<T, A>::iterator it = toIterate.begin(); it != toIterate.end(); ++it)
		{
		    if (*it != toIterate.back())
		        std::cout << safe_print(*it) << ", ";
		    else
		    	std::cout << safe_print(*it);
		}
		std::cout << "]";
	};

	// dummy template - string is not iterable
	template <typename E, typename A>
	void printContainer(std::_String_val<E, A> toIterate){};

	template <typename T, typename A, template <typename, typename> class C >
	void printNestedContainerHelper(C<T, A > toPrint)
	{
	    for (C<T, A >::iterator it = toPrint.begin(); it != toPrint.end(); ++it)
	        printNestedContainer(*it);
	};

	// dummy template - string is not iterable
	template <typename E, typename A>
	void printNestedContainerHelper(std::_String_val<E, A> toPrint){};

	template <typename T, template <typename, typename> class C >
	void printNestedContainer(C<T, std::allocator<T> > toPrint)
	{
		// Although we know that with this test will only call printContainer for a container of printable stuff we'll need code it in a safe way so the compiler doesn't find an error if it tries to deduce it for a non iterable container or a container of not printable things, let's say, a _String_val container which is not iterable or a container of container which is not printable
	    if (has_insertion_operator<T>::value)
	    {
	    	printContainer(toPrint);
	    }
	    // Kind of the same situation here, but only with the not iterable container case
	    else
	    {
	        std::cout << "[";
	        printNestedContainerHelper(toPrint);
	        std::cout << "]";
	    }
	};

	template <class M, class V>
	void MapToVec(const  M & m, V & v)
	{
		for (M::const_iterator it = m.begin(); it != m.end(); ++it)
		{
			v.push_back( it->second );
		}
	};

	template <class T, const size_t siz>
	class CArray
	{
		private:
			T array[siz];
		public:
			const size_t size() const { return siz; };

			template<typename R> T& operator[](R pos)
			{
				if (pos >= siz)
				{
					std::stringstream ss;
					ss << "Common::CArray: invalid index. ";
					throw(Common::MyException(ss.str()));
				}
				return array[pos];
			};
	};

	typedef CArray<double, 3> CArray3d;

}

#endif // COMMON_H

// cmake:sourcegroup=Helpers
