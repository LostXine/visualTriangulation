#ifndef _HELLO_
#define _HELLO_
#if defined(_MSC_VER)||defined(__CYGWIN__)||defined(__MINGW32__)
#	ifdef util_EXPORTS	//这个名字需要和项目名称util相符合，否则会报C4273
#		define HELLO_EXPORT __declspec(dllexport)
#	else
#		define HELLO_EXPORT __declspec(dllimport)
#	endif
#else
#	define HELLO_EXPORT
#endif
extern "C" class HELLO_EXPORT Hello
{
public:
	void sayhello(const char* name);
};

#endif //_HELLO_
