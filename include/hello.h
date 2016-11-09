#ifndef _HELLO_
#define _HELLO_
#if defined(_MSC_VER)||defined(__CYGWIN__)||defined(__MINGW32__)
#	ifdef HELLO_LIBRARY
#		define HELLO_EXPORT __declspec(dllexport)
#	else
#		define HELLO_EXPORT __declspec(dllimport)
#	endif
#else
#	define HELLO_EXPORT
#endif
class HELLO_EXPORT Hello
{
public:
	void sayhello(const char* name);
};

#endif //_HELLO_
