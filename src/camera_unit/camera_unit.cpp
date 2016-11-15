#include <cstdio>
#include <glog/logging.h>  
#include "camera_unit.h"

#include "rapidjson/rapidjson.h"
#include "rapidjson/document.h"
#include "rapidjson/reader.h"
#include "rapidjson/writer.h"
#include "rapidjson/pointer.h"
#include "rapidjson/stringbuffer.h"

using namespace rapidjson;

double parseDouble(Document &d,const char * p,int i)
{
double r = -1;
char q[32];
sprintf(q,"/%s/%d",p,i);
if(Value* _r = Pointer(q).Get(d)){r= _r->GetDouble();};
return r;
}

camera_unit::camera_unit(const char* js)
{
index = -1;
memset(img_path,0,sizeof(img_path));
Document d;
d.Parse(js);	
if (d.HasParseError())
{
LOG(ERROR)<<js<<" HasParseError";
return;
}
else
{
if (Value* _index = Pointer("/index").Get(d)){index = _index->GetInt();}
if (Value* _path = Pointer("/image path").Get(d)){memcpy(img_path,_path->GetString(),_path->GetStringLength());}
for(int i = 0;i<3;i++)
{
delta_R[i] = parseDouble(d,"R vector",i);
delta_T[i] = parseDouble(d,"T vector",i);
}
LOG(INFO)<<"Load {Index:"<<index<<" Path:"<<img_path<<" R:["<<delta_R[0]<<"|"<<delta_R[1]<<"|"<<delta_R[2]<<"] T:["<<delta_T[0]<<"|"<<delta_T[1]<<"|"<<delta_T[2]<<"]}";
}
}
