#ifndef _tri_manager_
#define _tri_manager_
#if defined(_MSC_VER)||defined(__CYGWIN__)||defined(__MINGW32__)
#	ifdef tri_manager_EXPORTS	//���������Ҫ����Ŀ����tri_manager����ϣ�����ᱨC4273
#		define tri_manager_EXPORT __declspec(dllexport)
#	else
#		define tri_manager_EXPORT __declspec(dllimport)
#	endif
#else
#	define tri_manager_EXPORT
#endif
extern "C" class tri_manager_EXPORT tri_manager
{
public:
	void sayhello(const char* name);
	void testOpenCV(const char* path);
};

#endif //_tri_manager_
