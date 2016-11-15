#include <tri_manager/tri_manager.h>
#include <cstdio>

int main(int argc, char** argv)
{
/*测试代码*/
/*
    tri_manager obj;
    obj.sayhello("world");
    obj.testOpenCV("path");
*/
printf("---TRIANGULATION MANAGER---\nBuild time:%s %s\n",__TIME__,__DATE__);

//打印参数环节：
printf("---PRINT PARA---\nTotal para:%d\n",argc);
for(int i = 0;i<argc;i++)
{
printf("Para %02d:%s\n",i,argv[i]);
}

//加载JSON环节


return 0;
}
